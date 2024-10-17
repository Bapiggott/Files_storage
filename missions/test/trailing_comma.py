"""import json

json_file = 'capture_30-0001.pcap.json'# Read the JSON file
with open(json_file, 'r') as file:
    data = json.load(file)

# Convert the data back to a JSON string without the trailing comma
json_str = json.dumps(data, indent=2)

# Write the updated JSON string back to the file

with open(f'{json_file}_2', 'w') as file:
    file.write(json_str)"""
'''import json

json_file = 'capture_30-0001.pcap.json'

try:
    with open(json_file, 'r') as file:
        data = json.load(file)
except json.JSONDecodeError as e:
    print(f"Error decoding JSON: {e}")
    raise  # Reraise the exception to see the full traceback

# Convert the data back to a JSON string without the trailing comma
json_str = json.dumps(data, indent=2)

# Write the updated JSON string back to the file
with open(f'{json_file}_2', 'w') as file:
    file.write(json_str)'''
import json


def remove_trailing_comma(json_file_path):
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

# Example usage
json_file_path = 'capture_30-0001.pcap.json'
remove_trailing_comma(json_file_path)
