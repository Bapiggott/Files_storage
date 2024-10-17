"""def process_large_json_file_comma(file_path, chunk_size=10000):
    with open(file_path, 'r+') as file:
        while True:
            # Read a chunk of the file
            chunk = file.read(chunk_size)

            if not chunk:
                # End of file
                break

            # Find the last open brace in the current chunk
            last_open_brace_index = chunk.rfind(',')

            if last_open_brace_index != -1:
                # Seek to the last open brace position in the file
                file.seek(file.tell() - (len(chunk) - last_open_brace_index))

                # Check if the next characters are ",,"
                next_chars = file.read(2)

                if ',' in next_chars:
                    # Delete the last open brace and the preceding comma
                    file.seek(file.tell() - 1)
                    file.truncate()

                    # Add "}]\n" after deleting the comma
                    file.write("}\n]\n")

            # Move to the next chunk
            file.seek(file.tell() - (chunk_size - last_open_brace_index))"""


def remove_incomplete_package2(file_path, output_file, chunk_size=1024*1024*10):
    with open(file_path, 'r') as source_file:
        with open(output_file, 'w') as dest_file:
            last_brace_index = None
            while True:
                chunk = source_file.read(chunk_size)
                if not chunk:
                    break

                # Find the index of the last '}'
                last_brace_index = chunk.rfind(',') # }')
                dest_file.write(chunk[:last_brace_index - 1])

            if last_brace_index is not None:
                # Append ']' to the last chunk
                dest_file.write(' }\n]')

    print(f"Removed incomplete packages from {file_path} and saved to {output_file}")


def remove_incomplete_package(file_path, output_file, chunk_size=1024*1024*10):
    with open(file_path, 'r') as source_file:
        with open(output_file, 'w') as dest_file:
            last_brace_index = None
            while True:
                chunk = source_file.read(chunk_size)
                if not chunk:
                    break

                # Find the index of the last '}'
                last_brace_index = chunk.rfind('"_index"') # }')
                dest_file.write(chunk[:last_brace_index - 3])

            if last_brace_index is not None:
                # Append ']' to the last chunk
                dest_file.write(']')

    print(f"Removed incomplete packages from {file_path} and saved to {output_file}")


# Example usage
file_path = 'capture_30-0001.pcap.json'
remove_incomplete_package(file_path, f'{file_path}_a.json')
remove_incomplete_package2(f'{file_path}_a.json', f'{file_path}_b.json')
