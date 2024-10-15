from typing import List, Tuple, Union, Optional
import re
from enum import Enum
import time
from threading import Thread
from queue import Queue

# Debug function to print debug information (currently just a placeholder)
def print_debug(*args):
    print(*args)

# Define a type alias for values that can be returned by MiniSpec
MiniSpecValueType = Union[int, float, bool, str, None]

# Function to evaluate a string value and return its corresponding MiniSpecValueType
def evaluate_value(value: str) -> MiniSpecValueType:
    if value.isdigit():
        return int(value)
    elif value.replace('.', '', 1).isdigit():
        return float(value)
    elif value == 'True':
        return True
    elif value == 'False':
        return False
    elif value == 'None' or len(value) == 0:
        return None
    else:
        return value.strip('\'"')

# Class to encapsulate a value and a replan flag for MiniSpec
class MiniSpecReturnValue:
    def __init__(self, value: MiniSpecValueType, replan: bool):
        self.value = value
        self.replan = replan

    def from_tuple(t: Tuple[MiniSpecValueType, bool]):
        return MiniSpecReturnValue(t[0], t[1])

    def default():
        return MiniSpecReturnValue(None, False)

    def __repr__(self) -> str:
        return f'value={self.value}, replan={self.replan}'

# SkillSet related classes
class SkillSetLevel(Enum):
    LOW = "low"
    HIGH = "high"

class SkillSet:
    def __init__(self, level="low", lower_level_skillset: 'SkillSet' = None):
        self.skills = {}
        self.level = SkillSetLevel(level)
        self.lower_level_skillset = lower_level_skillset

    def get_skill(self, skill_name: str) -> Optional['SkillItem']:
        skill = None
        if skill_name in self.skills:
            skill = self.skills[skill_name]
        elif skill_name in SkillItem.abbr_dict:
            skill = self.skills.get(SkillItem.abbr_dict[skill_name])
        return skill

    def add_skill(self, skill_item: 'SkillItem'):
        if skill_item.skill_name in self.skills:
            raise ValueError(f"A skill with the name '{skill_item.skill_name}' already exists.")
        if self.level == SkillSetLevel.HIGH and isinstance(skill_item, HighLevelSkillItem):
            if self.lower_level_skillset is not None:
                skill_item.set_skillset(self.lower_level_skillset, self)
            else:
                raise ValueError("Low-level skillset is not set.")
        self.skills[skill_item.skill_name] = skill_item

    def remove_skill(self, skill_name: str):
        if skill_name not in self.skills:
            raise ValueError(f"No skill found with the name '{skill_name}'.")
        del self.skills[skill_name]

    def __repr__(self) -> str:
        string = ""
        for skill in self.skills.values():
            string += f"{skill}\n"
        return string

# Continue with the rest of the SkillSet classes (LowLevelSkillItem, HighLevelSkillItem, etc.)
# ...

# Enum to represent different parsing states within the MiniSpec parser
class ParsingState(Enum):
    CODE = 0
    ARGUMENTS = 1
    CONDITION = 2
    LOOP_COUNT = 3
    SUB_STATEMENTS = 4

# Class to represent a MiniSpec program, which is a collection of statements
class MiniSpecProgram:
    def __init__(self, env: Optional[dict] = None) -> None:
        self.statements: List[Statement] = []
        self.depth = 0
        self.finished = False
        self.ret = False
        self.env = env if env is not None else {}
        self.current_statement = Statement(self.env)

    def parse(self, code_instance: Union[List[str], str], exec: bool = False) -> bool:
        for chunk in code_instance:
            if isinstance(chunk, str):
                code = chunk
            else:
                code = chunk.choices[0].delta.content
            if code is None or len(code) == 0:
                continue
            for c in code:
                if self.current_statement.parse(c, exec):
                    if len(self.current_statement.action) > 0:
                        print_debug("Adding statement: ", self.current_statement, exec)
                        self.statements.append(self.current_statement)
                    self.current_statement = Statement(self.env)
                if c == '{':
                    self.depth += 1
                elif c == '}':
                    if self.depth == 0:
                        self.finished = True
                        return True
                    self.depth -= 1
        return False
    
    def write_to_file(self, filename: str) -> None:
        with open(filename, 'w') as f:
            for statement in self.statements:
                f.write(str(statement) + ';\n')

    def __repr__(self) -> str:
        s = ''
        for statement in self.statements:
            s += f'{statement}; '
        return s

# Class to represent an individual statement in MiniSpec
class Statement:
    execution_queue: Queue['Statement'] = None
    low_level_skillset: SkillSet = None  # Low-level skillset reference
    high_level_skillset: SkillSet = None  # High-level skillset reference

    def __init__(self, env: dict) -> None:
        self.code_buffer: str = ''
        self.parsing_state: ParsingState = ParsingState.CODE
        self.condition: Optional[str] = None
        self.loop_count: Optional[int] = None
        self.action: str = ''
        self.allow_digit: bool = False
        self.executable: bool = False
        self.ret: bool = False
        self.sub_statements: Optional[MiniSpecProgram] = None
        self.env = env
        self.read_argument: bool = False

    def get_env_value(self, var) -> MiniSpecValueType:
        if var not in self.env:
            raise Exception(f'Variable {var} is not defined')
        return self.env[var]

    def parse(self, code: str, exec: bool = False) -> bool:
        for c in code:
            match self.parsing_state:
                case ParsingState.CODE:
                    if c == '?' and not self.read_argument:
                        self.action = 'if'
                        self.parsing_state = ParsingState.CONDITION
                    elif c == ';' or c == '}' or c == ')':
                        if c == ')':
                            self.code_buffer += c
                            self.read_argument = False
                        self.action = self.code_buffer
                        print_debug(f'SP Action: {self.code_buffer}')
                        self.executable = True
                        if exec and self.action != '':
                            self.execution_queue.put(self)
                        return True
                    else:
                        if c == '(':
                            self.read_argument = True
                        if c.isalpha() or c == '_':
                            self.allow_digit = True
                        self.code_buffer += c
                    if c.isdigit() and not self.allow_digit:
                        self.action = 'loop'
                        self.parsing_state = ParsingState.LOOP_COUNT
                case ParsingState.CONDITION:
                    if c == '{':
                        print_debug(f'SP Condition: {self.code_buffer}')
                        self.condition = self.code_buffer
                        self.code_buffer = ''
                        self.parsing_state = ParsingState.SUB_STATEMENTS
                        self.sub_statements = MiniSpecProgram(self.env)
                    else:
                        self.code_buffer += c
                case ParsingState.LOOP_COUNT:
                    if c == '{':
                        print_debug(f'SP Loop count: {self.code_buffer}')
                        self.loop_count = int(self.code_buffer)
                        self.code_buffer = ''
                        self.parsing_state = ParsingState.SUB_STATEMENTS
                        self.sub_statements = MiniSpecProgram(self.env)
                    else:
                        self.code_buffer += c
                case ParsingState.SUB_STATEMENTS:
                    if c == '}':
                        self.sub_statements.parse('}', exec)
                        return True
                    else:
                        self.sub_statements.parse(c, exec)
                case ParsingState.ARGUMENTS:
                    self.code_buffer += c
        return False
    
    def __repr__(self) -> str:
        return f'Statement(action={self.action})'

# Usage example (parsing code and writing it to a file):
if __name__ == "__main__":
    code = "if (x > 5) { loop 10 { y = y + 1; } } return y;"
    program = MiniSpecProgram()
    program.parse(code)
    program.write_to_file("output.txt")
