import os
import json
import shutil
import subprocess
from multiprocessing import Pool

def combine_json_files(input_directory, output_combined_file):
    json_contents = []

    for filename in os.listdir(input_directory):
        if filename.endswith(".json"):
            input_file_path = os.path.join(input_directory, filename)

            try:
                with open(input_file_path, 'r') as f:
                    lines = f.readlines()

                    for line in lines:
                        try:
                            file_content = json.loads(line)
                            json_contents.append(file_content)
                        except json.decoder.JSONDecodeError as e:
                            print(f"Error decoding JSON in file {input_file_path}: {e}")

            except Exception as ex:
                print(f"Error reading file {input_file_path}: {ex}")

    combined_content = []
    for content in json_contents:
        combined_content.extend(content)

    with open(output_combined_file, 'w') as output_file:
        json.dump(combined_content, output_file, indent=4)

def add_na_messages(json_data, context_size):
    for section_name, section_data in json_data.items():
        new_messages = [
            {key: None for key in section_data[0].keys()}
            for _ in range(context_size + 1)
        ]
        json_data[section_name] = new_messages + section_data
    return json_data

def convert_sets_to_lists(obj):
    if isinstance(obj, dict):
        return {key: convert_sets_to_lists(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_sets_to_lists(element) for element in obj]
    elif isinstance(obj, set):
        return list(obj)
    else:
        return obj

def create_sliding_window(content, window_size):
    sliding_window_data = []
    window = []

    for message in content:
        window.append(message)
        window = window[-window_size:]

        if len(window) == window_size:
            c_data = window[:-2]
            q_data = window[-2]
            a_data = window[-1]

            sliding_window_data.append({"Context": c_data, "Question": q_data, "Answer": a_data})

    return sliding_window_data

def tcp_case(entries, output_entries, previous_flag, previous_source, previous_destination):
    # Your tcp_case logic here...

def udp_case(entries, output_entries, previous_flag, previous_source, previous_destination):
    # Your udp_case logic here...

def mvl_case(entries, output_entries, previous_flag, previous_source, previous_destination):
    # Your mvl_case logic here...

def default(entries, output_entries, previous_flag, previous_source, previous_destination):
    # Your default logic here...

def switch_case(case, entries, output_entries, previous_flag, previous_source, previous_destination):
    switch_dict = {
        0: tcp_case,
        1: udp_case,
        2: mvl_case,
    }
    return switch_dict.get(case, default)(entries, output_entries, previous_flag, previous_source, previous_destination)

def process_data(entries, protocol_index):
    output_entries = []
    previous_flag = None
    previous_source = None
    previous_destination = None

    output_entries = switch_case(protocol_index, entries, output_entries, previous_flag, previous_source,
                                 previous_destination)

    return output_entries

def finalize_json(context_size, index, file):
    window_size = context_size + 2
    na_file = f"output_na_{protocol_vector[index]}.json"

    with open(file, "r") as f:
        json_data = json.load(f)

    json_data_with_na = add_na_messages(json_data, context_size)

    with open(na_file, "w") as f:
        json.dump(json_data_with_na, f, indent=2)

    with open(na_file, 'r') as f:
        data = json.load(f)

    output_file_json = f'output_file{protocol_vector[index]}.json'

    output_data = {}
    for key, entries in data.items():
        output_data[key] = process_data(entries, index)

    with open(output_file_json, 'w') as output_file:
        json.dump(output_data, output_file, indent=2)

    with open(output_file_json, 'r') as f:
        data = json.load(f)

    sliding_window_data = {file_name: create_sliding_window(content, window_size) for file_name, content in
                           data.items()}

    sliding_window_data = convert_sets_to_lists(sliding_window_data)

    output_file_name = f"sliding_window_size_{protocol_vector[index]}.json"
    with open(output_file_name, 'w') as f:
        json.dump(sliding_window_data, f, indent=2)

    flattened_conversations_json = f'flattened_conversations_{protocol_vector[index]}.json'
    with open(flattened_conversations_json, 'w') as f:
        json.dump(flattened_conversations, f, indent=2)

def filtered_jsons_wrapper(args):
    filtered_jsons(*args)

def run_pcap_splitter(pcap_file):
    command = f'~/Downloads/pcapplusplus-23.09-ubuntu-22.04-gcc-11.2.0-x86_64/bin/PcapSplitter -f "{pcap_file}" -o . -m connection'
    try:
        subprocess.run(command, shell=True, check=True)
        print(f"Processed {pcap_file}")
    except subprocess.CalledProcessError as e:
        print(f"Error processing {pcap_file}: {e}")

def process_pcaps():
    processed_directory = "processed_pcaps"
    if not os.path.exists(processed_directory):
        os.makedirs(processed_directory)

    pcap_files = [file for file in os.listdir(".") if file.endswith(".pcap")]

    for pcap_file in pcap_files:
        run_pcap_splitter(pcap_file)
        shutil.move(pcap_file, os.path.join(processed_directory, pcap_file))

    print("All pcap files processed and moved.")

def combine_and_finalize_wrapper(args):
    combine_and_finalize(*args)

def combine_and_finalize(output_directory, protocol_vector, index):
    output_combined_file = f'output_combined{protocol_vector}.json'
    combine_json_files(output_directory, output_combined_file)
    context_size = 4
    finalize_json(context_size, index, output_combined_file)

def process_directory(index, element, output_directory):
    make_dir(output_directory)
    filtered_jsons(element, output_directory)
    print(f"Processed index {index}")

def remove_trailing_comma(file_path, output_file, chunk_size=1024*1024*10):
    # Your remove_trailing_comma logic here...

def remove_incomplete_package(file_path, output_file, chunk_size=1024*1024*10):
    # Your remove_incomplete_package logic here...

def delete_file(file_path):
    try:
        os.remove(file_path)
        print(f"File '{file_path}' successfully deleted.")
    except OSError as e:
        print(f"Error deleting file '{file_path}': {e}")

def process_file(file_path):
    temp_file_path = f"{file_path}_temp.json"
    remove_incomplete_package(file_path, temp_file_path)
    remove_trailing_comma(temp_file_path, file_path)
    delete_file(temp_file_path)

def fix_json_format_parallel(directory_protocol, num_processes=6):
    # Your fix_json_format_parallel logic here...

if __name__ == "__main__":
    directory_protocol = ["./jsons1/Tcp_jsons"]
    protocol_vector = ["tcp"]
    output_directory_filtered = ["output_filtered_tcp"]
    jsons_output_directory = "jsons1"
    output_combined_file = "output_combined.json"
    target_directory = "./jsons1"
    # run_move_jsons(target_directory)

    output_directory = 'output_filtered_jsons2'
    print("Beginning to filter pcaps, gaining the fields")

    process_directory(1, directory_protocol[0], output_directory_filtered[0])
    num_cores = 5
    # Use concurrent.futures or multiprocessing.Pool here...

    for index, element in enumerate(protocol_vector):
        output_combined_file = f'output_combined{protocol_vector[index]}.json'
        combine_json_files(output_directory_filtered[index], output_combined_file)
        context_size = 4
        finalize_json(context_size, index, output_combined_file)

