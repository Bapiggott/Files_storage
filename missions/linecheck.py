import json

def read_specific_line(json_file_path, line_number):
    with open(json_file_path, 'r') as file:
        for current_line_number, line in enumerate(file, start=1):
            if current_line_number == line_number:
                try:
                    # Attempt to load the entire line as a JSON array
                    json_array = json.loads(line)
                    
                    # If the line is a JSON array, extract the specific JSON object
                    if isinstance(json_array, list):
                        json_data = json_array[0]  # Change index if needed
                        return json_data
                    else:
                        print(f"The line at {line_number} is not a JSON array.")
                        return None
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON on line {line_number}: {e}")
                    return None
    return None

# Example usage
json_file_path = './jsons1/Tcp_jsons/capture_66-0001.pcap.json'# './jsons1/Tcp_jsons/capture_68-0003.pcap.json'
line_number_to_read = 52869458

json_data_at_line = read_specific_line(json_file_path, line_number_to_read)

if json_data_at_line is not None:
    print(json_data_at_line)
else:
    print(f"Unable to read line {line_number_to_read} from the JSON file.")

