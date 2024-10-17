import re

class MiniSpecProgram:
    def __init__(self, code: str) -> None:
        self.code = code
        self.index = 0
        self.length = len(code)
        self.python_code = []

    def parse(self) -> None:
        while self.index < self.length:
            char = self.code[self.index]
            if char.isspace():
                self.index += 1
            elif char == '?':
                self.handle_conditional()
            elif char.isdigit():
                self.handle_loop()
            elif char == '_':
                self.handle_variable_assignment()
            elif char == '->':
                self.handle_return()
            elif char == '{' or char == '}':
                self.index += 1
            else:
                self.index += 1

    def handle_conditional(self) -> None:
        self.index += 1  # Skip '?'
        condition = self.parse_condition()
        self.python_code.append(f'if {condition}:')
        self.index += 1  # Skip '{'
        self.parse_block()
        self.index += 1  # Skip '}'

    def handle_loop(self) -> None:
        loop_count = self.parse_integer()
        self.python_code.append(f'for _ in range({loop_count}):')
        self.index += 1  # Skip '{'
        self.parse_block()
        self.index += 1  # Skip '}'

    def handle_variable_assignment(self) -> None:
        self.index += 1  # Skip '_'
        variable = self.parse_variable()
        self.index += 1  # Skip '='
        self.index += 1  # Skip possible space
        value = self.parse_value()
        self.python_code.append(f'{variable} = {value}')

    def handle_return(self) -> None:
        self.index += 2  # Skip '->'
        self.index += 1  # Skip possible space
        value = self.parse_value()
        self.python_code.append(f'return {value}')

    def parse_block(self) -> None:
        while self.index < self.length:
            char = self.code[self.index]
            if char == '}':
                break
            elif char == '{':
                self.index += 1
                self.parse_block()
            elif char.isdigit():
                self.handle_loop()
            elif char == '?':
                self.handle_conditional()
            elif char == '_':
                self.handle_variable_assignment()
            elif char == '->':
                self.handle_return()
            else:
                self.index += 1

    def parse_condition(self) -> str:
        left_operand = self.parse_operand()
        comparator = self.parse_comparator()
        right_operand = self.parse_operand()
        return f'{left_operand} {comparator} {right_operand}'

    def parse_operand(self) -> str:
        if self.code[self.index] == '_':
            return self.parse_variable()
        return self.parse_value()

    def parse_integer(self) -> int:
        match = re.match(r'\d+', self.code[self.index:])
        if match:
            self.index += len(match.group(0))
            return int(match.group(0))
        print(f"Warning: No match found for integer at index {self.index}")
        return 0

    def parse_variable(self) -> str:
        match = re.match(r'\d+', self.code[self.index:])
        if match:
            self.index += len(match.group(0))
            return '_' + match.group(0)
        print(f"Warning: No match found for variable at index {self.index}")
        return ''

    def parse_function_call(self) -> str:
        function_name = re.match(r'[a-zA-Z_]\w*', self.code[self.index:])
        if function_name:
            function_name = function_name.group(0)
            self.index += len(function_name)
            if self.code[self.index] == '(':
                self.index += 1
                arguments = self.parse_arguments()
                self.index += 1  # Skip ')'
                return f'{function_name}({arguments})'
            return function_name
        print(f"Warning: No match found for function call at index {self.index}")
        return ''

    def parse_arguments(self) -> str:
        args = []
        while self.index < self.length and self.code[self.index] != ')':
            if self.code[self.index].isspace():
                self.index += 1
                continue
            args.append(self.parse_value())
            if self.code[self.index] == ',':
                self.index += 1
        return ', '.join(args)

    def parse_value(self) -> str:
        if self.code[self.index] == '_':
            return self.parse_variable()
        match = re.match(r'\d+(\.\d+)?', self.code[self.index:])
        if match:
            self.index += len(match.group(0))
            return match.group(0)
        print(f"Warning: No match found for value at index {self.index}")
        return ''

    def parse_comparator(self) -> str:
        if self.code[self.index:self.index+2] in ['==', '!=']:
            comp = self.code[self.index:self.index+2]
            self.index += 2
            return comp
        comp = self.code[self.index]
        self.index += 1
        return comp

    def write_output(self) -> None:
        with open('output.py', 'w') as f:
            f.write('\n'.join(self.python_code))

if __name__ == '__main__':
    code = """
_1 = {6}; ? _1 > 5 { _2 = _2 + 1; }
"""
    mini_spec_program = MiniSpecProgram(code)
    mini_spec_program.parse()
    mini_spec_program.write_output()
    print("Python code has been written to output.py")
