from typing import List, Optional, Union
class MiniSpecProgram:
    def __init__(self, env: Optional[dict] = None) -> None:
        self.statements: List['Statement'] = []
        self.depth = 0
        self.finished = False
        self.env = env if env is not None else {}
        self.current_statement = Statement(self.env)

    def parse(self, code_instance: Union[List[str], str], exec: bool = False) -> bool:
        print(f"Starting to parse the code: {code_instance}")
        for chunk in code_instance:
            code = chunk if isinstance(chunk, str) else chunk.choices[0].delta.content
            if code is None or len(code) == 0:
                continue
            for c in code:
                if self.current_statement.parse(c, exec):
                    if self.current_statement.sub_statements:
                        print(f"Sub-statements completed: {self.current_statement.sub_statements}")
                    if len(self.current_statement.action) > 0 or self.current_statement.sub_statements:
                        print(f"Adding statement: {self.current_statement}")
                        self.statements.append(self.current_statement)
                    self.current_statement = Statement(self.env)
                if c == '{':
                    self.depth += 1
                    print(f"Entering block: current depth is {self.depth}")
                elif c == '}':
                    if self.depth == 0:
                        self.finished = True
                        return True
                    self.depth -= 1
                    print(f"Exiting block: current depth is {self.depth}")
        print(f"Finished parsing: {self.statements}")
        return False

    def write_to_file(self, filename: str) -> None:
        with open(filename, 'w') as f:
            for statement in self.statements:
                f.write(str(statement) + ';\n')
        print(f"Program written to {filename}")

    def __repr__(self) -> str:
        return '; '.join([str(statement) for statement in self.statements])


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
        print(f"Parsing character: {code}, state: {self.parsing_state}")
        for c in code:
            if self.parsing_state == 'CODE':
                if c == '?':
                    print("Detected condition, switching state to CONDITION")
                    self.parsing_state = 'CONDITION'
                elif c == '-':
                    print("Detected return arrow (->)")
                    self.parsing_state = 'RETURN'
                elif c.isdigit():
                    print(f"Detected loop count: {c}")
                    self.loop_count = int(c)
                    self.parsing_state = 'LOOP'
            elif self.parsing_state == 'CONDITION':
                if c == '{':
                    self.condition = self.code_buffer.strip()
                    print(f"Condition set: {self.condition}")
                    self.sub_statements = MiniSpecProgram(self.env)
                    self.code_buffer = ''
                    self.parsing_state = 'SUB_STATEMENTS'
                else:
                    self.code_buffer += c
            elif self.parsing_state == 'LOOP':
                if c == '{':
                    print(f"Starting loop with count {self.loop_count}")
                    self.sub_statements = MiniSpecProgram(self.env)
                    self.parsing_state = 'SUB_STATEMENTS'
            elif self.parsing_state == 'RETURN':
                self.action = 'return ' + self.code_buffer.strip()
                print(f"Return action: {self.action}")
                return True
            elif self.parsing_state == 'SUB_STATEMENTS':
                if c == '}':
                    print(f"Sub-statements finished: {self.sub_statements}")
                    return True
                else:
                    if not self.sub_statements:
                        self.sub_statements = MiniSpecProgram(self.env)
                    self.sub_statements.parse(c, exec)
        return False

    def __repr__(self) -> str:
        if self.action.startswith('return'):
            return f'-> {self.action}'
        elif self.loop_count is not None:
            return f'{self.loop_count} {{ {self.sub_statements} }}'
        elif self.condition:
            return f'? {self.condition} {{ {self.sub_statements} }}'
        else:
            return self.action


if __name__ == "__main__":
    code = "? _1 > 5 { 10 { _2 = _2 + 1; } } -> _2;"
    program = MiniSpecProgram()
    program.parse(code)
    program.write_to_file("output.txt")
