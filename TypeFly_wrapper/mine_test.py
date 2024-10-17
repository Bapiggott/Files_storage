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
            'rp': 're_plan',
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

        self.await_function = [
            'connect_drone',
            'ensure_armed_and_taken_off',
            'moving_drone',
            'get_heading',
            'move_forward',
            'move_backward',
            'set_heading_with_velocity',
            'move_left',
            'move_right',
            'changing_elevation',
            'move_up',
            'move_down',
            'turn_cw',
            'turn_ccw',
            'move_in_circle',
            'delay',
            'orienting',
            'approach',
            'scan',
            'scan_abstract',
            'goto'
        ]

    def add_statement(self, statement: str) -> None:
        """Add MiniSpec statement and its Python equivalent."""
        self.statements.append(statement)
        python_stmt = self.translate_to_python(statement)
        self.python_code.append(python_stmt)

    def translate_to_python(self, minispec_stmt: str) -> str:
        """Translate MiniSpec-like statement to Python equivalent."""
        minispec_stmt = minispec_stmt.strip()
        if not minispec_stmt:
            return ""  # Handle empty statements gracefully

        # Handle loop syntax
        if minispec_stmt[0].isdigit():
            loop_count = int(minispec_stmt.strip('{').strip())
            return self.get_indent() + f"for _ in range({loop_count}):"

        # Handle conditions
        if minispec_stmt.startswith('?'):
            condition = minispec_stmt[1:].strip()
            condition = condition.replace('&', ' and ').replace('|', ' or ')
            return self.get_indent() + f"if {condition}:"

        # Handle return statement
        elif minispec_stmt.startswith('->'):
            return self.get_indent() + f"return {minispec_stmt[2:].strip()}"

        # Handle assignment statements
        if '=' in minispec_stmt:
            lhs, rhs = minispec_stmt.split('=', 1)
            lhs = lhs.strip()
            rhs = rhs.strip()

            # Process RHS for function abbreviations
            for abbr, func_name in self.abbreviations.items():
                if rhs.startswith(abbr + '('):
                    if func_name in self.await_function:
                        rhs = 'await ' + func_name + '(' + rhs[len(abbr) + 1:-1] + ')'
                    else:
                        rhs = func_name + '(' + rhs[len(abbr) + 1:-1] + ')'
                    break  # Exit after replacing the abbreviation

            return self.get_indent() + f"{lhs} = {rhs}"

        # Check for function call with abbreviation
        for abbr, func_name in self.abbreviations.items():
            if minispec_stmt.startswith(abbr + '('):
                # Replace abbreviation with function name
                if func_name in self.await_function:
                    return self.get_indent() + "await " + func_name + '(' + minispec_stmt[len(abbr) + 1:-1] + ')'
                else:
                    return self.get_indent() + func_name + '(' + minispec_stmt[len(abbr) + 1:-1] + ')'

        # Check for function call with full function name
        func_name_full = minispec_stmt.split('(', 1)[0]
        if func_name_full in self.functions:
            if func_name_full in self.await_function:
                return self.get_indent() + "await " + func_name_full + '(' + minispec_stmt[
                                                                             len(func_name_full) + 1:-1] + ')'
            else:
                return self.get_indent() + func_name_full + '(' + minispec_stmt[len(func_name_full) + 1:-1] + ')'

        # Handle other statements
        return self.get_indent() + minispec_stmt

    def get_indent(self) -> str:
        """Get the current indentation level for Python code."""
        return ("\t" * self.indentation_level) + "\t"

    def write_output(self, filename: str) -> None:
        """Write MiniSpec and equivalent Python code to the output file."""
        with open(filename, 'w') as f:
            f.write('from functions import *\nimport asyncio\n\n')
            f.write('async def main():\n')
            f.write('\n'.join(self.python_code))
            f.write("\nasyncio.run(main())\n")

    def parse(self, code: str) -> None:
        """Simulate parsing and add statements."""
        statement = ""  # To accumulate characters into a statement
        i = 0
        while i < len(code):
            char = code[i]
            if char == ';':  # End of a statement
                if statement.strip():
                    self.add_statement(statement.strip())
                    statement = ""  # Reset statement buffer
                i += 1
            elif char == '{':
                if statement.strip():
                    self.add_statement(statement.strip())
                    statement = ""  # Reset statement buffer
                self.indentation_level += 1
                i += 1
            elif char == '}':
                if statement.strip():
                    self.add_statement(statement.strip())
                    statement = ""  # Reset statement buffer
                self.indentation_level -= 1
                i += 1
            else:
                statement += char
                i += 1
        if statement.strip():
            self.add_statement(statement.strip())  # Add final statement

        # Write the output to the file
        self.write_output('output.py')


# Example use case
if __name__ == '__main__':
    code = "tu(90);?iv('apple')==True&iv('orange'){l('Yes');->True}l('No');->False;?s('bottle')==True{g('bottle');_2=oh('bottle');l(_2);tp};"#4{_1=ox($1);?_1>0.6{tc(15)};?_1<0.4{tu(15)};_2=ox($1);?_2<0.6&_2>0.4{->True}}->False;"#"5 { _1 = p('Any animal target here?');?_1 != False { l(_1);-> True; }tc(30);} -> False;"
    program = MiniSpecProgram()
    program.parse(code)
    print("Program output written to 'output.py'")
