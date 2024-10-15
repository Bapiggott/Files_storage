from typing import List, Optional

class MiniSpecProgram:
    def __init__(self, env: Optional[dict] = None) -> None:
        self.statements: List[str] = []  # To store MiniSpec statements
        self.python_code: List[str] = []  # To store equivalent Python code
        self.env = env if env is not None else {}
        self.indentation_level = 0  # To track indentation in Python code

    def add_statement(self, statement: str) -> None:
        """Add MiniSpec statement and its Python equivalent."""
        self.statements.append(statement)
        python_stmt = self.translate_to_python(statement)
        self.python_code.append(python_stmt)

    def translate_to_python(self, minispec_stmt: str) -> str:
        """Translate MiniSpec-like statement to Python equivalent."""
        # Handle condition
        if minispec_stmt.startswith('?'):
            condition = minispec_stmt[1:].strip()
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
            f.write('MiniSpec Program Output:\n')
            f.write('\n'.join(self.statements))
            f.write('\n\nEquivalent Python Code:\n')
            f.write('\n'.join(self.python_code))

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
        self.write_output('output.txt')

# Example use case
if __name__ == '__main__':
    code = "? _1 > 5 { _2 = _2 + 1; } -> _2;"
    program = MiniSpecProgram()
    program.parse(code)
    print("Program output written to 'output.txt'")
