import time
from typing import List, Optional
import re



class MiniSpecProgram:
    def __init__(self, env: Optional[dict] = None) -> None:
        self.statements: List[str] = []  # To store MiniSpec statements
        self.python_code: List[str] = []  # To store equivalent Python code
        self.env = env if env is not None else {}
        self.indentation_level = 1  # Start with indentation level 1 for 'async def main()'

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
            'delay': 'delay(seconds: int)',
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
        python_stmt = self.translate_to_python(statement)
        if python_stmt:
            self.python_code.append(python_stmt)

    def translate_to_python(self, minispec_stmt: str) -> str:
        import re
        minispec_stmt = minispec_stmt.strip()
        if not minispec_stmt:
            return ""

        # Handle loop syntax
        if minispec_stmt[0].isdigit():
            loop_count = int(minispec_stmt.strip('{').strip())
            return self.get_indent() + f"for _ in range({loop_count}):"

        # Handle conditions
        if minispec_stmt.startswith('?'):
            condition = minispec_stmt[1:].strip()
            condition = condition.replace('&', ' and ').replace('|', ' or ')
            # Process abbreviations in the condition
            condition = self.replace_abbreviations(condition)
            # Find all function calls in the condition
            func_calls = re.findall(r'(\w+\(.*?\))', condition)
            for call in func_calls:
                func_name = call.split('(')[0]
                if func_name in self.await_function:
                    # Replace function call with 'await' prefixed
                    condition = condition.replace(call, f'await {call}')
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
            rhs = self.replace_abbreviations(rhs)
            # Check if RHS is a function call
            func_match = re.match(r'^(\w+)\s*\(.*\)$', rhs)
            if func_match:
                func_name = func_match.group(1)
                if func_name in self.await_function:
                    rhs = 'await ' + rhs
            return self.get_indent() + f"{lhs} = {rhs}"

        # Check for function call with abbreviation
        for abbr, func_name in self.abbreviations.items():
            if minispec_stmt.startswith(abbr + '('):
                # Replace abbreviation with function name
                if func_name in self.await_function:
                    return self.get_indent() + "await " + func_name + '(' + minispec_stmt[len(abbr) + 1:-1] + ')'
                else:
                    return self.get_indent() + func_name + '(' + minispec_stmt[len(abbr) + 1:-1] + ')'

        # Check for function call without arguments
        if minispec_stmt in self.abbreviations:
            func_name = self.abbreviations[minispec_stmt]
            if func_name in self.await_function:
                return self.get_indent() + "await " + func_name + "()"
            else:
                return self.get_indent() + func_name + "()"

        # Process abbreviations in the entire statement
        minispec_stmt = self.replace_abbreviations(minispec_stmt)

        # Handle other statements
        return self.get_indent() + minispec_stmt

    def replace_abbreviations(self, s: str) -> str:
        import re
        for abbr, func_name in self.abbreviations.items():
            # Match abbreviation when it's a standalone word or followed by '('
            pattern = r'\b' + re.escape(abbr) + r'(\b|\()'
            # If func_name is a function without arguments
            if func_name in self.functions and self.functions[func_name].endswith('()'):
                # If followed by '(', keep it (e.g., function call with arguments)
                if re.search(r'\b' + re.escape(abbr) + r'\(', s):
                    replacement = func_name + '('
                else:
                    replacement = func_name + '()'
            else:
                replacement = func_name + r'\1'  # Keep the '(' or word boundary
            s = re.sub(pattern, replacement, s)
        return s

    def get_indent(self) -> str:
        """Get the current indentation level for Python code."""
        return '    ' * self.indentation_level  # 4 spaces per indentation level


    def write_output(self, filename: str) -> None:
        """Write MiniSpec and equivalent Python code to the output file."""
        with open(filename, 'w') as f:
            f.write('from functions import *\n') # _copy import *\n')
            f.write('import asyncio\n')
            f.write('import datetime\n')
            f.write('import json\n\n')
            # Include the original MiniSpec code
            escaped_original_code = self.original_code.replace('"""', '\\"\\"\\"')
            f.write(f'original_code = """{escaped_original_code}"""\n\n')
            f.write('async def main():\n')
            f.write('    await connect_drone()\n')
            f.write('    await ensure_armed_and_taken_off()\n')
            f.write('    await return_to_start_position()\n')  # Add this line
            f.write('    # Start the print_status_text task\n')
            f.write('    status_task = asyncio.create_task(print_status_text(drone))\n\n')
            f.write('    try:\n')
            for line in self.python_code:
                f.write(f"    {line}\n")
            f.write('\n')
            f.write('    finally:\n')
            f.write('        await stop_offboard_mode()\n')
            f.write('        status_task.cancel()\n')
            f.write('        try:\n')
            f.write('            await status_task\n')
            f.write('        except asyncio.CancelledError:\n')
            f.write('            pass\n\n')
            # Include code to run main() and log the output
            f.write("if __name__ == '__main__':\n")
            f.write('    result = asyncio.run(main())\n')
            f.write('    current_datetime = datetime.datetime.now().isoformat()\n')
            f.write('    with open(__file__, \'r\') as f_in:\n')
            f.write('        translated_code = f_in.read()\n')
            f.write('    log_data = {\n')
            f.write('        \'date\': current_datetime,\n')
            f.write('        \'original_code\': original_code,\n')
            f.write('        \'translated_code\': translated_code,\n')
            f.write('        \'output\': result\n')
            f.write('    }\n')
            f.write('    with open(\'execution_log.json\', \'a\') as log_file:\n')
            f.write('        log_file.write(json.dumps(log_data) + \'\\n\')\n')

    def parse(self, code: str) -> None:
        """Simulate parsing and add statements."""
        self.original_code = code
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


# Example use case
if __name__ == '__main__':
    import subprocess  # Add this import

    # List of MiniSpec code snippets
    """codes = [
        '''"4{_1=ox($1);?_1>0.6{tc(15)};?_1<0.4{tu(15)};_2=ox($1);?_2<0.6&_2>0.4{->True}}->False;",
        "5 { _1 = p('Any animal target here?');?_1 != False { l(_1);-> True; } tc(30);} -> False;",
        "8{_1=p($1);?_1!=False{->_1}tc(45)}->False;"'''
    ]"""
    codes = [
        #"_1=s('person');?_1==True{g('person');->True}->False;",
        "tu(90);?iv('apple')==True&iv('orange'){l('Yes');->True}l('No');->False;",
        "?s('bottle')==True{g('bottle');_2=oh('bottle');l(_2);tp};"
    ]

    for idx, code in enumerate(codes, start=1):
        program = MiniSpecProgram()
        program.parse(code)
        output_filename = f'output_{idx}.py'
        program.write_output(output_filename)
        print(f"Program output written to '{output_filename}'")

        try:
            print(f"Executing '{output_filename}'...")
            result = subprocess.run(['python3', output_filename], check=True)
            print(f"Execution of '{output_filename}' completed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"An error occurred while executing '{output_filename}': {e}")
        print("Waiting...")
        time.sleep(100)

        # Reset the program state for the next code snippet
        program.statements = []
        program.python_code = []
        program.indentation_level = 1

