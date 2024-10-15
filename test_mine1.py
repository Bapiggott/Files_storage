from typing import List, Union, Optional

# MiniSpec grammar-compliant parser

# Class to represent a MiniSpec program, which is a collection of statements
class MiniSpecProgram:
    def __init__(self, env: Optional[dict] = None) -> None:
        self.statements: List[Statement] = []
        self.depth = 0
        self.finished = False
        self.env = env if env is not None else {}
        self.current_statement = Statement(self.env)

    def parse(self, code_instance: Union[List[str], str], exec: bool = False) -> bool:
        for chunk in code_instance:
            code = chunk if isinstance(chunk, str) else chunk.choices[0].delta.content
            if code is None or len(code) == 0:
                continue
            for c in code:
                if self.current_statement.parse(c, exec):
                    if len(self.current_statement.action) > 0:
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
        return '; '.join([str(statement) for statement in self.statements])

# Class to represent an individual statement in MiniSpec
class Statement:
    def __init__(self, env: dict) -> None:
        self.code_buffer: str = ''
        self.parsing_state: str = 'CODE'
        self.condition: Optional[str] = None
        self.loop_count: Optional[int] = None
        self.action: str = ''
        self.env = env
        self.sub_statements: Optional[MiniSpecProgram] = None

    def parse(self, code: str, exec: bool = False) -> bool:
        for c in code:
            if self.parsing_state == 'CODE':
                if c == '?':
                    self.parsing_state = 'CONDITION'
                elif c == '->':
                    self.action = 'return'
                    return True
                elif c.isdigit():
                    self.action = 'loop'
                    self.loop_count = int(c)
                    self.parsing_state = 'LOOP'
            elif self.parsing_state == 'CONDITION':
                if c == '{':
                    self.condition = self.code_buffer.strip()
                    self.code_buffer = ''
                    self.sub_statements = MiniSpecProgram(self.env)
                    self.parsing_state = 'SUB_STATEMENTS'
                else:
                    self.code_buffer += c
            elif self.parsing_state == 'LOOP':
                if c == '{':
                    self.sub_statements = MiniSpecProgram(self.env)
                    self.parsing_state = 'SUB_STATEMENTS'
            elif self.parsing_state == 'SUB_STATEMENTS':
                if c == '}':
                    return True
                else:
                    self.sub_statements.parse(c, exec)

        return False

    def __repr__(self) -> str:
        if self.action == 'return':
            return f'-> {self.code_buffer.strip()}'
        elif self.action == 'loop':
            return f'{self.loop_count} {{ {self.sub_statements} }}'
        elif self.action == 'if':
            return f'? {self.condition} {{ {self.sub_statements} }}'
        else:
            return self.action

# Usage example
if __name__ == "__main__":
    code = "? _1 > 5 { 10 { _2 = _2 + 1; } } -> _2;"
    program = MiniSpecProgram()
    program.parse(code)
    program.write_to_file("output.txt")
