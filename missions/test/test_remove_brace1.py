import os
import json


def remove_incomplete_package1(file_path, output_file):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    last_brace_index = None

    # Find the index of the last '}'
    for i in range(len(lines) - 1, -1, -1):
        if '}' in lines[i]:
            last_brace_index = i
            break

    # If a '}' is found, remove everything after it and add ']'
    if last_brace_index is not None:
        lines = lines[:last_brace_index + 1]
        lines.append(']')

        with open(output_file, 'w') as file:
            file.writelines(lines)


def remove_trailing_comma1(json_file_path):
    if not json_file_path.endswith('.json'):
        return  # Not a JSON file

    with open(json_file_path, 'r') as file:
        lines = file.readlines()

    # Find the line with the last comma
    for i in range(len(lines) - 1, -1, -1):
        line = lines[i].strip()
        if line.endswith(',') and line != ',':
            # Remove the last comma from the line
            lines[i] = line[:-1] + '\n'
            break

    # Write the updated lines back to the file
    with open(json_file_path, 'w') as file:
        file.writelines(lines)



"""def remove_incomplete_package2(file_path, output_file, chunk_size=1024*1024*10):
    with open(file_path, 'r') as source_file:
        with open(output_file, 'w') as dest_file:
            last_brace_index = None
            while True:
                chunk = source_file.read(chunk_size)
                if not chunk:
                    break

                # Find the index of the last '}'
                last_brace_index = chunk.rfind('}')
                if last_brace_index is None:
                    dest_file.write(chunk)
            if last_brace_index is not None:
                lines = lines[:last_brace_index + 1]
                lines.append(']')
            if last_brace_index is not None:
                chunk.
                dest_file.write()
                dest_file.write(']')

    print(f"Removed incomplete packages from {file_path} and saved to {output_file}")"""


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


def remove_trailing_comma(json_file_path, chunk_size=1024*1024*10):
    if not json_file_path.endswith('.json'):
        return  # Not a JSON file

    with open(json_file_path, 'r') as source_file:
        with open(json_file_path + '.tmp', 'w') as dest_file:
            trailing_comma_found = False
            while True:
                chunk = source_file.read(chunk_size)
                if not chunk:
                    break

                # Find the line with the last comma
                last_comma_index = chunk.rfind(',')
                dest_file.write(chunk)

                if last_comma_index != -1:
                    trailing_comma_found = True

            if not trailing_comma_found:
                print(f"No trailing comma found in {json_file_path}")
                return
    # Replace the original file with the temporary file
    os.rename(json_file_path + '.tmp', json_file_path)
    print(f"Removed trailing commas from {json_file_path}")


def remove_trailing_brace(json_file_path, chunk_size=1024*1024*10):
    if not json_file_path.endswith('.json'):
        return  # Not a JSON file

    with open(json_file_path, 'r') as source_file:
        with open(json_file_path + '.tmp', 'w') as dest_file:
            trailing_comma_found = False
            while True:
                chunk = source_file.read(chunk_size)
                if not chunk:
                    break

                # Find the line with the last comma
                last_comma_index = chunk.rfind('}')
                dest_file.write(chunk)

                if last_comma_index != -1:
                    trailing_comma_found = True

            if not trailing_comma_found:
                print(f"No trailing brace found in {json_file_path}")
                return


# Replace 'your_file_path' with the path to your specific file
file_path = 'capture_9-0007.pcap.json'
# file_path = 'capture_30-0001.pcap.json'
file_path1 = f'{file_path}_2.json'
remove_incomplete_package(file_path, file_path1)
# remove_trailing_brace(file_path1)
# remove_trailing_comma(file_path1)
