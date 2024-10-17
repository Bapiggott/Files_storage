import json

input_file_path = 'capture_30-0001.pcap_2.json'
output_file_path = 'your_output_file.json'

def remove_second_and_third_last_chars(input_file, output_file, chunk_size=1024*1024*10):
    with open(input_file, 'r') as source_file:
        with open(output_file, 'w') as dest_file:
            last_brace_index = None
            while True:
                chunk = source_file.read(chunk_size)
                if not chunk:
                    break

                # Find the index of the last '}' in the chunk
                last_brace_index = chunk.rfind('}')

                # Write the chunk excluding the 2nd and 3rd to last characters
                dest_file.write(chunk[:last_brace_index - 3] + chunk[last_brace_index - 1:])

            if last_brace_index is not None:
                # Append '}' to the last chunk
                dest_file.write('}')

    print(f"Removed 2nd and 3rd to last characters from {input_file} and saved to {output_file}")

# Example usage
remove_second_and_third_last_chars(output_file_path, f'{input_file_path}_output.json')
