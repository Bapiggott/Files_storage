import ijson

def process_large_json_file(file_path, chunk_size=10000):
    with open(file_path, 'r+') as file:
        stack = []
        buffer = []

        # Read the file content
        content = file.read()

        # Find the last open brace
        last_open_brace_index = content.rfind('{')

        if last_open_brace_index != -1:
            print("hi")
            # Seek to the last open brace
            file.seek(last_open_brace_index)

            # Check if the next characters are "],"
            next_chars = file.read(6)
            #print(next_chars)
            # print(file.read(6))

            #print(file.read(last_open_brace_index -2))
            #print(file.read(last_open_brace_index - 3))
            print(file.read(last_open_brace_index-2))
            if ']' in next_chars:
                print("hola")
                # Delete the last open brace and the preceding comma
                file.seek(last_open_brace_index-2)
                file.truncate()


def process_large_json_file_comma(file_path, chunk_size=10000):
    with open(file_path, 'r+') as file:
        stack = []
        buffer = []

        # Read the file content
        content = file.read()

        # Find the last open brace
        last_open_brace_index = content.rfind(',')

        if last_open_brace_index != -1:
            print("hi")
            # Seek to the last open brace
            file.seek(last_open_brace_index)

            # Check if the next characters are ",,"
            next_chars = file.read(6)
            #print(next_chars)
            # print(file.read(6))

            #print(file.read(last_open_brace_index -2))
            #print(file.read(last_open_brace_index - 3))
            print(file.read(last_open_brace_index-2))
            if ',' in next_chars:
                print("hola")
                # Delete the last open brace and the preceding comma
                file.seek(last_open_brace_index-1)
                file.truncate()


# Example usage
file_path = 'capture_9-0007.pcap.json_2.json'
process_large_json_file_comma(file_path)

