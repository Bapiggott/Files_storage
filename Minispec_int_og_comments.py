from typing import List, Tuple, Union, Optional
import re
from enum import Enum
import time
from threading import Thread
from queue import Queue
from openai import ChatCompletion, Stream
from .skillset import SkillSet
from .utils import split_args, print_t

# Debug function to print debug information (currently just a placeholder)
def print_debug(*args):
    print(*args)
    # pass  # This is commented out; uncomment to suppress debug output

# Define a type alias for values that can be returned by MiniSpec
MiniSpecValueType = Union[int, float, bool, str, None]

# Function to evaluate a string value and return its corresponding MiniSpecValueType
def evaluate_value(value: str) -> MiniSpecValueType:
    # Check if the value is an integer
    if value.isdigit():
        return int(value)
    # Check if the value is a float
    elif value.replace('.', '', 1).isdigit():
        return float(value)
    # Check if the value is a boolean True
    elif value == 'True':
        return True
    # Check if the value is a boolean False
    elif value == 'False':
        return False
    # Check if the value is None or an empty string
    elif value == 'None' or len(value) == 0:
        return None
    # Otherwise, return the value as a stripped string (removing quotes)
    else:
        return value.strip('\'"')

# Class to encapsulate a value and a replan flag for MiniSpec
class MiniSpecReturnValue:
    def __init__(self, value: MiniSpecValueType, replan: bool):
        self.value = value  # The actual value being stored
        self.replan = replan  # A flag to indicate whether replanning is required

    # Factory method to create a MiniSpecReturnValue from a tuple
    def from_tuple(t: Tuple[MiniSpecValueType, bool]):
        return MiniSpecReturnValue(t[0], t[1])
    
    # Factory method to return a default value (None, no replan)
    def default():
        return MiniSpecReturnValue(None, False)
    
    # String representation of the MiniSpecReturnValue for debugging
    def __repr__(self) -> str:
        return f'value={self.value}, replan={self.replan}'

# Enum to represent different parsing states within the MiniSpec parser
class ParsingState(Enum):
    CODE = 0  # Default state, parsing the code itself
    ARGUMENTS = 1  # Parsing arguments within a function call
    CONDITION = 2  # Parsing a condition for an if statement
    LOOP_COUNT = 3  # Parsing the loop count for a loop
    SUB_STATEMENTS = 4  # Parsing sub-statements within blocks like loops or ifs

# Class to represent a MiniSpec program, which is a collection of statements
class MiniSpecProgram:
    def __init__(self, env: Optional[dict] = None) -> None:
        self.statements: List[Statement] = []  # List to store parsed statements
        self.depth = 0  # Depth level of nested statements
        self.finished = False  # Flag to indicate if parsing is complete
        self.ret = False  # Flag to indicate if a return was encountered
        self.env = env if env is not None else {}  # Environment to store variable values
        self.current_statement = Statement(self.env)  # Current statement being parsed

    # Method to parse a stream of code (either a list of strings or a stream of chunks)
    def parse(self, code_instance: Stream[ChatCompletion.ChatCompletionChunk] | List[str], exec: bool = False) -> bool:
        for chunk in code_instance:
            if isinstance(chunk, str):
                code = chunk  # Directly assign string chunks
            else:
                code = chunk.choices[0].delta.content  # Extract content from non-string chunks
            if code is None or len(code) == 0:
                continue
            for c in code:
                # If current statement is fully parsed, add it to the statements list
                if self.current_statement.parse(c, exec):
                    if len(self.current_statement.action) > 0:
                        print_debug("Adding statement: ", self.current_statement, exec)
                        self.statements.append(self.current_statement)
                    # Start parsing a new statement
                    self.current_statement = Statement(self.env)
                if c == '{':
                    self.depth += 1  # Increase depth for nested blocks
                elif c == '}':
                    if self.depth == 0:
                        self.finished = True  # Indicate that parsing is finished
                        return True
                    self.depth -= 1  # Decrease depth for nested blocks
        return False
    
    # Method to evaluate all parsed statements in the program
    def eval(self) -> MiniSpecReturnValue:
        print_debug(f'Eval program: {self}, finished: {self.finished}')
        ret_val = MiniSpecReturnValue.default()  # Start with a default return value
        count = 0
        while not self.finished:
            if len(self.statements) <= count:
                time.sleep(0.1)  # Wait for more statements to be added if not finished
                continue
            ret_val = self.statements[count].eval()  # Evaluate the current statement
            if ret_val.replan or self.statements[count].ret:
                print_debug(f'RET from {self.statements[count]} with {ret_val} {self.statements[count].ret}')
                self.ret = True  # Indicate that a return was encountered
                return ret_val
            count += 1
        if count < len(self.statements):
            for i in range(count, len(self.statements)):
                ret_val = self.statements[i].eval()  # Evaluate remaining statements
                if ret_val.replan or self.statements[i].ret:
                    print_debug(f'RET from {self.statements[i]} with {ret_val} {self.statements[i].ret}')
                    self.ret = True
                    return ret_val
        return ret_val
    
    # String representation of the MiniSpecProgram for debugging
    def __repr__(self) -> str:
        s = ''
        for statement in self.statements:
            s += f'{statement}; '
        return s

# Class to represent an individual statement in MiniSpec
class Statement:
    execution_queue: Queue['Statement'] = None  # Shared execution queue for all statements
    low_level_skillset: SkillSet = None  # Low-level skillset reference
    high_level_skillset: SkillSet = None  # High-level skillset reference

    def __init__(self, env: dict) -> None:
        self.code_buffer: str = ''  # Buffer to store the current code being parsed
        self.parsing_state: ParsingState = ParsingState.CODE  # Initial parsing state
        self.condition: Optional[str] = None  # Condition for if statements
        self.loop_count: Optional[int] = None  # Loop count for loop statements
        self.action: str = ''  # Action to be executed
        self.allow_digit: bool = False  # Flag to allow digits in variable names
        self.executable: bool = False  # Flag to indicate if the statement is executable
        self.ret: bool = False  # Flag to indicate if a return was encountered
        self.sub_statements: Optional[MiniSpecProgram] = None  # Sub-statements for loops or conditions
        self.env = env  # Environment to store variable values
        self.read_argument: bool = False  # Flag to track if arguments are being read

    # Method to get the value of a variable from the environment
    def get_env_value(self, var) -> MiniSpecValueType:
        if var not in self.env:
            raise Exception(f'Variable {var} is not defined')
        return self.env[var]

    # Method to parse a single statement from a code string
    def parse(self, code: str, exec: bool = False) -> bool:
        for c in code:
            match self.parsing_state:
                case ParsingState.CODE:
                    # Start parsing a condition if '?' is encountered and no arguments are being read
                    if c == '?' and not self.read_argument:
                        self.action = 'if'
                        self.parsing_state = ParsingState.CONDITION
                    # End parsing the statement on semicolon, closing brace, or closing parenthesis
                    elif c == ';' or c == '}' or c == ')':
                        if c == ')':
                            self.code_buffer += c
                            self.read_argument = False
                        self.action = self.code_buffer
                        print_debug(f'SP Action: {self.code_buffer}')
                        self.executable = True  # Mark the statement as executable
                        if exec and self.action != '':
                            self.execution_queue.put(self)
                        return True
                    else:
                        if c == '(':
                            self.read_argument = True
                        if c.isalpha() or c == '_':
                            self.allow_digit = True  # Allow digits in variable names after an alpha character
                        self.code_buffer += c  # Add the character to the code buffer
                    # If a digit is encountered and not allowed, start parsing a loop
                    if c.isdigit() and not self.allow_digit:
                        self.action = 'loop'
                        self.parsing_state = ParsingState.LOOP_COUNT
                case ParsingState.CONDITION:
                    if c == '{':
                        print_debug(f'SP Condition: {self.code_buffer}')
                        self.condition = self.code_buffer  # Store the parsed condition
                        self.code_buffer = ''
                        self.parsing_state = ParsingState.SUB_STATEMENTS
                        self.sub_statements = MiniSpecProgram(self.env)  # Parse sub-statements
                    else:
                        self.code_buffer += c
                case ParsingState.LOOP_COUNT:
                    if c == '{':
                        print_debug(f'SP Loop count: {self.code_buffer}')
                        self.loop_count = int(self.code_buffer)  # Store the loop count
                        self.code_buffer = ''
                        self.parsing_state = ParsingState.SUB_STATEMENTS
                        self.sub_statements = MiniSpecProgram(self.env)  # Parse sub-statements
                    else:
                        self.code_buffer += c
                case ParsingState.SUB_STATEMENTS:
                    if c == '}':
                        self.sub_statements.parse('}', exec)  # Finish parsing sub-statements
                        return True
                    else:
                        self.sub_statements.parse(c, exec)  # Continue parsing sub-statements
                case ParsingState.ARGUMENTS:
                    self.code_buffer += c  # Add to the code buffer if parsing arguments
        return False
    
    # Method to evaluate the parsed statement and return its result
    def eval(self) -> MiniSpecReturnValue:
        print_debug(f'Eval statement: {self.action}')
        if self.action.startswith('if'):
            if self.condition:
                condition = self.condition.strip('() ')
                condition_value = evaluate_value(self.get_env_value(condition))  # Evaluate the condition
                print_debug(f'Condition: {condition_value} for {self.condition}')
                if condition_value:
                    return self.sub_statements.eval()  # Execute sub-statements if condition is true
        elif self.action.startswith('loop'):
            for _ in range(self.loop_count):
                ret_val = self.sub_statements.eval()  # Execute sub-statements for each loop iteration
                if ret_val.replan:
                    return ret_val
        elif self.action.startswith('return'):
            ret_val = evaluate_value(self.code_buffer.strip('return '))
            return MiniSpecReturnValue(ret_val, False)  # Return the evaluated value
        else:
            pass
        return MiniSpecReturnValue.default()  # Default return value

    # String representation of the Statement for debugging
    def __repr__(self) -> str:
        return f'Statement(action={self.action})'
