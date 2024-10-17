import json

# Your JSON data
input_file_path = 'capture_30-0001.pcap.json'

try:
    with open(input_file_path, "r") as file:
        json_data = json.load(file)
        print("JSON data loaded successfully.")
except json.JSONDecodeError as e:
    print(f"JSON decoding error: {e}")
    with open(input_file_path, "r") as file:
        # Print the content of the file for debugging purposes
        print(file.read())
        # Optionally, you can exit the program or handle the error in another way
        exit()

# Rest of your code
# Function to remove the last field from a JSON object
def remove_last_field(json_obj):
    if "_source" in json_obj:
        source = json_obj["_source"]
        if "layers" in source:
            layers = source["layers"]
            if "data" in layers:
                del layers["data"]
    return json_obj

# Apply the function to each JSON object
modified_json_data = [remove_last_field(obj) for obj in json_data]

# Print the modified JSON data
print(json.dumps(modified_json_data, indent=2))
