from typing import List, Optional

class MiniSpecProgram:
    def __init__(self, env: Optional[dict] = None) -> None:
        self.statements: List[str] = []  # To store MiniSpec statements
        self.python_code: List[str] = []  # To store equivalent Python code
        self.env = env if env is not None else {}
        self.indentation_level = 0  # To track indentation in Python code

        # Define abbreviations and their corresponding function names
        self.abbreviations = {
            's': 'scan',
            'sa': 'scan_abstract',
            'o': 'orienting',
            'a': 'approach',
            'g': 'goto',
            'mf': 'move_forward',
            'mb': 'move_backward',
            'ml': 'move_left',
            'mr': 'move_right',
            'mu': 'move_up',
            'md': 'move_down',
            'tc': 'turn_cw',
            'tu': 'turn_ccw',
            'mi': 'move_in_circle',
            'd': 'delay',
            'iv': 'is_visible',
            'ox': 'object_x',
            'oy': 'object_y',
            'ow': 'object_width',
            'oh': 'object_height',
            'od': 'object_dis',
            'p': 'probe',
            'l': 'log',
            'tp': 'take_picture',
            'rp': 're_plan'
        }

        # Define function signatures
        self.functions = {
            'scan': 'scan(object_name: str)',
            'scan_abstract': 'scan_abstract(question: str)',
            'orienting': 'orienting(object_name: str)',
            'approach': 'approach()',
            'goto': 'goto(object_name: str)',
            'move_forward': 'move_forward(distance: int)',
            'move_backward': 'move_backward(distance: int)',
            'move_left': 'move_left(distance: int)',
            'move_right': 'move_right(distance: int)',
            'move_up': 'move_up(distance: int)',
            'move_down': 'move_down(distance: int)',
            'turn_cw': 'turn_cw(degrees: int)',
            'turn_ccw': 'turn_ccw(degrees: int)',
            'move_in_circle': 'move_in_circle(cw: bool)',
            'delay': 'delay(milliseconds: int)',
            'is_visible': 'is_visible(object_name: str)',
            'object_x': 'object_x(object_name: str)',
            'object_y': 'object_y(object_name: str)',
            'object_width': 'object_width(object_name: str)',
            'object_height': 'object_height(object_name: str)',
            'object_dis': 'object_dis(object_name: str)',
            'probe': 'probe(question: str)',
            'log': 'log(text: str)',
            'take_picture': 'take_picture()',
            're_plan': 're_plan()'
        }

    def add_statement(self, statement: str) -> None:
        """Add MiniSpec statement and its Python equivalent."""
        self.statements.append(statement)
        python_stmt = self.translate_to_python(statement)
        self.python_code.append(python_stmt)

    def translate_to_python(self, minispec_stmt: str) -> str:
        """Translate MiniSpec-like statement to Python equivalent."""
        # Replace abbreviations with full function names
        for abbr, func_name in self.abbreviations.items():
            minispec_stmt = minispec_stmt.replace(abbr + '(', func_name + '(')

        # Handle condition
        if minispec_stmt.startswith('?'):
            condition = minispec_stmt[1:].strip()
            # Replace logical operators with Python equivalents with spaces around them
            condition = condition.replace('&', ' and ').replace('|', ' or ')
            return self.get_indent() + f"if {condition}:"
        # Handle return statement
        elif minispec_stmt.startswith('->'):
            return self.get_indent() + f"return {minispec_stmt[2:].strip()}"
        # Handle blocks or loops
        elif minispec_stmt.startswith('{'):
            self.indentation_level += 1
            return ""
        elif minispec_stmt.startswith('}'):
            self.indentation_level -= 1
            return ""
        else:
            # Default to normal statement handling (like assignments)
            return self.get_indent() + minispec_stmt

    def get_indent(self) -> str:
        """Get the current indentation level for Python code."""
        return "    " * self.indentation_level

    def write_output(self, filename: str) -> None:
        """Write MiniSpec and equivalent Python code to the output file."""
        with open(filename, 'w') as f:
            f.write('from functions import *\n\n')  # Import functions
            f.write('\n'.join(self.python_code))  # Write the translated Python code

    def parse(self, code: str) -> None:
        """Simulate parsing and add statements."""
        statement = ""  # To accumulate characters into a statement
        for char in code:
            if char == ';':  # End of a statement
                self.add_statement(statement.strip())
                statement = ""  # Reset statement buffer
            elif char in '{}':  # Handle block braces
                if statement.strip():
                    self.add_statement(statement.strip())
                self.add_statement(char)
                statement = ""  # Reset after a block character
            else:
                statement += char  # Accumulate characters
        if statement.strip():
            self.add_statement(statement.strip())  # Add final statement

        # Write the output to the file
        self.write_output('output.py')

# Example use case
if __name__ == '__main__':
    code = "tu(90);?iv('apple')==True&iv('orange'){l('Yes');->True}l('No');->False;"
    program = MiniSpecProgram()
    program.parse(code)
    print("Program output written to 'output.py'")

