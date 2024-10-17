import os

def delete_empty_json_files(directory):
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            if os.path.getsize(file_path) == 0:
                print(f"Deleting {filename}")
                os.remove(file_path)

# Example usage:
directory_to_clean = "."
delete_empty_json_files(directory_to_clean)
