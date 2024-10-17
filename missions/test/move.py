"""def process_json_file(json_file_path):
    with open(json_file_path, 'r') as source_file:
        with open(json_file_path + '.tmp', 'w') as dest_file:
            trailing_comma_found = False
            while True:
                prev_chunk = chunk
                chunk = source_file.read(1024)  # Adjust the chunk size as needed
                content = dest_file.read()
                last_open_comma_index = content.rfind(',')
                if not chunk:
                    break
                if last_open_comma_index != -1:
                    print("hi")
                    # Seek to the last open brace
                    dest_file.seek(last_open_comma_index)

                    # Check if the next characters are "],"
                    next_chars = dest_file.read(6)
                    print(dest_file.read(last_open_comma_index - 2))
                    if ']' in next_chars:
                        print("hola")
                        # Delete the last open brace and the preceding comma
                        dest_file.seek(last_open_comma_index - 2)
                        dest_file.truncate()
                dest_file.write(chunk)
            print(chunk)
# Example usage
json_file_path = 'capture_9-0007.pcap.json_2.json'
process_json_file(json_file_path)

import os

# Replace the original file with the temporary one
os.replace('capture_9-0007.pcap.json_2.json.tmp', 'capture_9-0007.pcap.json_2.json')
"""
def process_json_file(json_file_path):
    with open(json_file_path, 'r') as source_file:
        with open(json_file_path + '.tmp', 'w') as dest_file:
            trailing_comma_found = False
            prev_chunk = ''
            while True:
                chunk = source_file.read(1024)  # Adjust the chunk size as needed
                if not chunk:
                    break

                # Combine the previous and current chunks
                combined_chunk = prev_chunk + chunk

                # Check for trailing comma in the combined chunk
                last_open_comma_index = combined_chunk.rfind(',')
                if last_open_comma_index != -1:
                    print("hi")
                    # Seek to the last open brace in dest_file
                    dest_file.seek(last_open_comma_index)

                    # Check if the next characters are "],"
                    next_chars = dest_file.read(6)
                    print(dest_file.read(last_open_comma_index - 2))
                    if ']' in next_chars:
                        print("hola")
                        # Delete the last open brace and the preceding comma
                        dest_file.seek(last_open_comma_index - 2)
                        dest_file.truncate()

                # Write the combined chunk to dest_file
                dest_file.write(combined_chunk)

                # Store the current chunk for the next iteration
                prev_chunk = chunk

            print(chunk)

# Example usage
json_file_path = 'capture_9-0007.pcap.json_2.json'
process_json_file(json_file_path)

import os

# Replace the original file with the temporary one
os.replace('capture_9-0007.pcap.json_2.json.tmp', 'capture_9-0007.pcap.json_2.json')
