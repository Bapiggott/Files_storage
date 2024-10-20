import os
import json
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor
from tqdm import tqdm
import re
from multiprocessing import Pool
import concurrent.futures
import ijson
import argparse
import math
import pandas as pd
import json
import concurrent.futures


def process_dict_ml(data_dict, current_frame, prefix=''):
    # Processes MavLink-related data dictionaries
    result = current_frame
    if result is None:
        result = current_frame
    for key, value in data_dict.items():
        full_key = f"{prefix}_{key}" if prefix else key

        if isinstance(value, dict):
            process_dict_ml(value, result, prefix=full_key)
        else:
            # Extract the portion of the key after the last period
            key_string = key.rsplit('.', 1)[-1]
            current_frame[key_string] = value

    return result


def process_dict_udp(data_dict, prefix='', result=None):
    # Processes UDP-related data dictionaries, filtering out specific keys.
    if result is None:
        result = {}
    for key, value in data_dict.items():
        full_key = f"{prefix}_{key}" if prefix else key

        if isinstance(value, dict):
            process_dict_udp(value, prefix=full_key, result=result)
        else:
            # Extract the portion of the key after the last period
            key_string = key.rsplit('.', 1)[-1]
            """if key_string != "payload":
                result[key_string] = value"""
            if key_string not in ["port"]:
                result[key_string] = value

    return result


def delete_empty_json_files(directory):
    # Deletes JSON files that are empty within a specified directory.
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            file_path = os.path.join(directory, filename)
            if os.path.getsize(file_path) == 0:
                print(f"Deleting {filename}")
                os.remove(file_path)


def process_dict_ml_ports(data_dict, prefix='', result=None):
    # Specifically targets port information within multi-layered dictionaries.
    if result is None:
        result = {}
    for key, value in data_dict.items():
        full_key = f"{prefix}_{key}" if prefix else key

        if isinstance(value, dict):
            process_dict_ml_ports(value, prefix=full_key, result=result)
        else:
            # Extract the portion of the key after the last period
            key_string = key.rsplit('.', 1)[-1]
            if key_string == "srcport" or key_string == "dstport":
                result[key_string] = value

    return result


def process_dict_tcp(data_dict, prefix='', result=None):
    # Filters and processes TCP packet data, extracting specified fields.
    if result is None:
        result = {}
    valid_key_strings = ["srcport", "dstport", "len", "seq", "ack", "str"]
    wanted_key_strings = ["sport", "dport", "len", "seq", "ack", "flags"]
    for key, value in data_dict.items():
        full_key = f"{prefix}_{key}" if prefix else key

        if isinstance(value, dict):
            process_dict_tcp(value, prefix=full_key, result=result)
        else:
            # Extract the portion of the key after the last period
            key_string = key.rsplit('.', 1)[-1]
            if key_string in valid_key_strings:
                index = valid_key_strings.index(key_string)
                if index == 5:
                    matches = re.findall(r'\b[A-Z]+\b', value)
                    result[wanted_key_strings[5]] = matches[0]
                else:
                    result[wanted_key_strings[index]] = value

    return result


def filtered_jsons(input_directory, output_directory, Context_size, type_index, chunk_size=100):
    # Filters JSON files based on specified network protocol and creates contextual datasets.

    # Create the output directory if it doesn't exist
    make_dir(output_directory)
    item_source = ['tcp', 'udp', 'mavlink_proto']
    for filename in os.listdir(input_directory):
        if filename.endswith(".json"):
            input_file_path = os.path.join(input_directory, filename)
            output_file_path = os.path.join(output_directory, f"filtered_comma_{filename}")
            with open(input_file_path, 'rb') as file:
                print(input_file_path)
                with open(output_file_path, 'a') as output_file:
                    output_file.write('[\n')
                    jsonobj = ijson.items(file, f'item._source.layers.{item_source[type_index]}', use_float=True)
                    first_item = True  # Flag to track if it's the first item
                    counter = 0
                    if item_source[type_index] == 'tcp':
                        for item in jsonobj:
                            if not first_item:
                                output_file.write(",\n")
                            try:
                                mavlink_proto_dict = item  # ["_source"]["layers"]["tcp"]
                                filtered_data = process_dict_tcp(mavlink_proto_dict)
                                if counter == 0:
                                    counter = 1
                                    for _ in range(Context_size):
                                        try:
                                            new_message = {key: None for key in filtered_data.keys()}
                                            if first_item:
                                                first_item = False
                                            json.dump(new_message, output_file, indent=4)
                                            output_file.write(',\n')

                                        except Exception as e:
                                            print(f"Error processing {file}: {e}")
                                        except Exception as ex:
                                            print(item)
                                            continue
                                first_item = False
                                json.dump(filtered_data, output_file, indent=4)
                            except Exception as ex:
                                print(item)
                                continue
                        # with open(output_file_path, 'a') as output_file:
                        output_file.write("\n]")
                    elif item_source[type_index] == 'mavlink_proto':
                        for item in jsonobj:
                            if not first_item:
                                output_file.write(",\n")
                            try:
                                mavlink_proto_dict = item  # ["_source"]["layers"]["tcp"]
                                filtered_data = process_dict_udp(mavlink_proto_dict)
                                if counter == 0:
                                    counter = 1
                                    for _ in range(Context_size):
                                        try:
                                            new_message = {key: None for key in filtered_data.keys()}
                                            if first_item:
                                                first_item = False
                                            json.dump(new_message, output_file, indent=4)
                                            output_file.write(',\n')

                                        except Exception as e:
                                            print(f"Error processing {file}: {e}")
                                        except Exception as ex:
                                            print(item)
                                            continue
                                first_item = False
                                json.dump(filtered_data, output_file, indent=4)
                            except Exception as ex:
                                print(item)
                                continue

                        # with open(output_file_path, 'a') as output_file:
                        output_file.write("\n]")
                    elif item_source[type_index] == 'udp':
                        for item in jsonobj:
                            if not first_item:
                                output_file.write(",\n")
                            try:
                                mavlink_proto_dict = item
                                results = process_dict_udp(mavlink_proto_dict)
                                mavlink_proto_dict = item
                                filtered_data = process_dict_ml(mavlink_proto_dict, results)
                                if counter == 0:
                                    counter = 1
                                    for _ in range(Context_size):
                                        try:
                                            new_message = {key: None for key in filtered_data.keys()}
                                            if first_item:
                                                first_item = False
                                            json.dump(new_message, output_file, indent=4)
                                            output_file.write(',\n')

                                        except Exception as e:
                                            print(f"Error processing {file}: {e}")
                                        except Exception as ex:
                                            print(item)
                                            continue
                                first_item = False
                                json.dump(filtered_data, output_file, indent=4)
                            except Exception as ex:
                                print(item)
                                continue
                        # with open(output_file_path, 'a') as output_file:
                        output_file.write("\n]")


def filtered_jsons_all(input_directory, Context_size, chunk_size=100):
    for index in range(len(directory_protocol)):
        make_dir(directory_protocol[index])
    item_source = ['tcp', 'udp', 'mavlink_proto']
    for filename in os.listdir(input_directory):
        for index in range(len(protocol_vector)):
            # print("HI")
            if filename.endswith(".json") and (protocol_vector[index] in filename):
                input_file_path = os.path.join(input_directory, filename)
                output_file_path = os.path.join(directory_protocol[index], f"filtered_comma_{filename}")
                with open(input_file_path, 'rb') as file:
                    print(input_file_path)
                    with open(output_file_path, 'a') as output_file:
                        output_file.write('[\n')
                        jsonobj = ijson.items(file, f'item._source.layers.{item_source[index]}', use_float=True)
                        first_item = True  # Flag to track if it's the first item
                        counter = 0
                        if item_source[index] == 'tcp':
                            for item in jsonobj:
                                if not first_item:
                                    output_file.write(",\n")
                                try:
                                    mavlink_proto_dict = item  # ["_source"]["layers"]["tcp"]
                                    filtered_data = process_dict_tcp(mavlink_proto_dict)
                                    if counter == 0:
                                        counter = 1
                                        for _ in range(Context_size):
                                            try:
                                                new_message = {key: None for key in filtered_data.keys()}
                                                if first_item:
                                                    first_item = False
                                                json.dump(new_message, output_file, indent=4)
                                                output_file.write(',\n')

                                            except Exception as e:
                                                print(f"Error processing {file}: {e}")
                                            except Exception as ex:
                                                print(item)
                                                continue
                                    first_item = False
                                    json.dump(filtered_data, output_file, indent=4)
                                except Exception as ex:
                                    print(item)
                                    continue
                            # with open(output_file_path, 'a') as output_file:
                            output_file.write("\n]")
                        elif item_source[index] == 'mavlink_proto':
                            for item in jsonobj:
                                if not first_item:
                                    output_file.write(",\n")
                                try:
                                    mavlink_proto_dict = item  # ["_source"]["layers"]["tcp"]
                                    filtered_data = process_dict_udp(mavlink_proto_dict)
                                    if counter == 0:
                                        counter = 1
                                        for _ in range(Context_size):
                                            try:
                                                new_message = {key: None for key in filtered_data.keys()}
                                                if first_item:
                                                    first_item = False
                                                json.dump(new_message, output_file, indent=4)
                                                output_file.write(',\n')

                                            except Exception as e:
                                                print(f"Error processing {file}: {e}")
                                            except Exception as ex:
                                                print(item)
                                                continue
                                    first_item = False
                                    json.dump(filtered_data, output_file, indent=4)
                                except Exception as ex:
                                    print(item)
                                    continue
                            # with open(output_file_path, 'a') as output_file:
                            output_file.write("\n]")
                        elif item_source[index] == 'udp':
                            for item in jsonobj:
                                if not first_item:
                                    output_file.write(",\n")
                                try:
                                    mavlink_proto_dict = item
                                    results = process_dict_udp(mavlink_proto_dict)
                                    mavlink_proto_dict = item
                                    filtered_data = process_dict_ml(mavlink_proto_dict, results)
                                    if counter == 0:
                                        counter = 1
                                        for _ in range(Context_size):
                                            try:
                                                new_message = {key: None for key in filtered_data.keys()}
                                                if first_item:
                                                    first_item = False
                                                json.dump(new_message, output_file, indent=4)
                                                output_file.write(',\n')

                                            except Exception as e:
                                                print(f"Error processing {file}: {e}")
                                            except Exception as ex:
                                                print(item)
                                                continue
                                    first_item = False
                                    json.dump(filtered_data, output_file, indent=4)
                                except Exception as ex:
                                    print(item)
                                    continue
                            # with open(output_file_path, 'a') as output_file:
                            output_file.write("\n]")


def convert_pcap_to_json(input_pcap, output_json):
    # Uses the Tshark utility to convert a single PCAP file to JSON format.
    try:
        tshark_command = f'tshark -r "{input_pcap}" -T json > "{output_json}"'
        subprocess.run(tshark_command, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error converting {input_pcap}: {e}")


def pcaps2jsons(output_directory):
    # Converts multiple PCAP files to JSON, parallelizing the process.

    # Create the output directory if it doesn't exist
    input_directory = "processed_pcaps"
    make_dir(output_directory)

    # Get a list of all the pcap files in the input directory
    pcap_files = [file for file in os.listdir(input_directory) if file.endswith(".pcap")]

    # Create a progress bar to track the conversion process
    with tqdm(total=len(pcap_files), desc="Converting pcaps to json") as pbar, \
            ThreadPoolExecutor() as executor:
        # Use ThreadPoolExecutor to parallelize the conversion process
        futures = []
        for pcap_file in pcap_files:
            input_pcap_path = os.path.join(input_directory, pcap_file)
            output_json_file = os.path.join(output_directory, f"{pcap_file}.json")
            futures.append(executor.submit(convert_pcap_to_json, input_pcap_path, output_json_file))

        # Wait for all tasks to complete
        for future in tqdm(concurrent.futures.as_completed(futures), total=len(futures), desc="Processing"):
            future.result()
            pbar.update(1)

    print("Conversion complete.")


def create_sliding_window_for_directory(directory_path, output_file_path, window_size, protocol_type):
    # Applies a sliding window technique to the data.
    for filename in os.listdir(directory_path):
        if filename.endswith(".json"):  # assuming the files are JSON
            file_path = os.path.join(directory_path, filename)
            create_sliding_window(file_path, output_file_path, window_size, protocol_type)


def create_sliding_window(file, output_file_path, window_size, protocol_type):
    window = []
    temp_window = []
    counter = 0
    filename = os.path.basename(file)
    output_file_path = output_file_path + '/' + filename.replace(".json", f"_slidding_window_{protocol_type}.json")
    with open(file, 'rb') as data:
        print(file)
        with open(output_file_path, 'a+') as output_file:
            jsonobj = ijson.items(data, 'item')
            # output_file.write('[\n')
            i = 0
            counter = 0
            '''for _ in jsonobj:
                counter += 1
            print(f'Amount of items: {counter}')'''
            jsonobj1 = ijson.items(data, 'item')
            first_item = True
            for item in jsonobj1:
                # print(item)
                sliding_window_data = []
                try:
                    window.append(item)
                    window = window[-window_size:]

                    if len(window) == window_size:
                        c_data = window[:-2]
                        q_data = window[-2]
                        a_data = window[-1]
                        sliding_window_data.append({"Context": c_data, "Question": q_data, "Answer": a_data})
                        json_string = json.dumps(sliding_window_data, indent=4)
                        # Remove the first and last characters (braces)
                        json_string = json_string[1:-1]
                        # print(json_string)
                        # with open(output_file_path, 'a') as output_file:
                        if not first_item:
                            output_file.write(',')
                        else:
                            output_file.write('[\n')
                            first_item = False
                        output_file.write(json_string)
                        # json.dump(sliding_window_data, output_file, indent=4)
                        """if i < counter - 1:
                            output_file.write(',')  # Add a comma after each item except the last one
                            # output_file.write('\n')
                        else:
                            output_file.write(']')
                            print('(])')
                        i += 1"""
                        # print(i)
                        # output_file.write(',')

                except Exception as e:
                    print(f"Error processing item: {e}")
            output_file.write(']')
        print("Created sliding window")


def run_pcap_splitter(pcap_file, output_dir):
    command = f'~/Downloads/pcapplusplus-23.09-ubuntu-22.04-gcc-11.2.0-x86_64/bin/PcapSplitter -f "{pcap_file}" -o "{output_dir}" -m connection'
    try:
        subprocess.run(command, shell=True, check=True)
        print(f"Processed {pcap_file}")
    except subprocess.CalledProcessError as e:
        print(f"Error processing {pcap_file}: {e}")


def process_pcaps(input_directory):
    processed_directory = ["unprocessed_pcaps", "processed_pcaps"]
    for index in range(len(processed_directory)):
        make_dir(processed_directory[index])

    pcap_files = [file for file in os.listdir(input_directory) if file.endswith(".pcap")]

    for pcap_file in pcap_files:
        input_path = os.path.join(input_directory, pcap_file)
        run_pcap_splitter(input_path, processed_directory[1])
        output_path = os.path.join(processed_directory[0], pcap_file)
        shutil.move(input_path, output_path)

    print("All pcap files processed and moved.")


def delete_file(file_path):
    try:
        os.remove(file_path)
        print(f"File '{file_path}' successfully deleted.")
    except OSError as e:
        print(f"Error deleting file '{file_path}': {e}")


def json2csv(filename, directory, protocol_type):
    # Converts JSON formatted data into CSV format, including context, question, and answer formats.
    try:
        # Read the JSON file
        with open(f'{directory}/{filename}', "r") as json_file:
            data = json.load(json_file)

        formatted_data = []

        # Loop through each record in the list
        for record in data:
            formatted_record = {
                "Context": record["Context"],
                "Question": record["Question"],
                "Answer": record["Answer"]
            }
            formatted_data.append(formatted_record)

        # Create a DataFrame from the formatted data
        df = pd.DataFrame(formatted_data)

        # Perform your operations on the DataFrame
        df = df.fillna("")

        text_col = []
        for _, row in df.iterrows():
            prompt = "Provided below is a question detailing key attributes of a UDP packet. Preceding this question is a series of UDP packets, which together form the context. Please continue the sequence by providing the subsequent UDP packet that follows the question, serving as the answer. \n\n"
            question = str(row["Question"])
            context = str(row["Context"])
            answer = str(row["Answer"])

            text = prompt + "### Context:\n" + context + "\n### Question\n" + question + "\n### Answer:\n" + answer

            text_col.append(text)

        df.loc[:, "text"] = text_col

        # Save the DataFrame as CSV
        csv_output = f'csvs_{protocol_type}'
        make_dir(csv_output)
        df.to_csv(f"./csvs_{protocol_type}/{filename}.csv", index=False)
        print(f"Processed file: {filename}")

    except json.JSONDecodeError as e:
        print(f"Error decoding JSON in file {filename}: {e}")


def process_files_json2csv(protocol_type):
    directory = f'./{protocol_type}/'
    with concurrent.futurdatasets_es.ProcessPoolExecutor() as executor:
        # Process each file concurrently
        futures = [executor.submit(json2csv, filename, directory, protocol_type) for filename in os.listdir(directory)
                   if filename.endswith(".json")]

        # Wait for all futures to complete
        concurrent.futures.wait(futures)


def divide(size, protocol_type, directory_input):
    # Divides large datasets into smaller chunks.

    """with open(input_file_path, 'a') as file:
        file.write(']')
        print('added ] to file')"""
    output_dir = f'datasets_{protocol_type}'
    make_dir(output_dir)
    slidding_window_jsons = [file for file in os.listdir(directory_input) if
                             file.endswith(f"_slidding_window_{protocol_type}.pcap")]
    for file in slidding_window_jsons:
        with open(file, 'rb') as file:
            # print(input_file_path)
            first_items = []
            # size = 50000
            total_datasets = 0
            jsonobj = ijson.items(file, 'item', use_float=True)
            total_datasets = sum(1 for _ in jsonobj)
            num_files = math.ceil(total_datasets / size)
            print(f'Total number of entries: {total_datasets}\nTotal number of files that\'ll be created: {num_files}')
            for _ in range(60):  # total_datasets):
                # with open(f'sub_dataset_{index}.json', 'a') as output_file:
                # output_file.write('[\n')
                first_items.append(True)
            # num_files = 60
            jsonobj = ijson.items(file, 'item', use_float=True)
            counter = 0
            for item in jsonobj:
                counter += 1
                index = counter
                # for index, item in jsonobj:
                print(f'index: {index}')
                try:
                    file_index = index % num_files
                    with open(f'{output_dir}/sub_dataset_{file_index}.json', 'a') as output_file:
                        if not first_items[file_index]:
                            output_file.write(",\n")
                        else:  # first_items[index]:
                            output_file.write('[\n')
                            first_items[file_index] = False
                        json_string = json.dumps(item, indent=4)
                        # json_string = json_string[1:-1]
                        output_file.write(json_string)
                except Exception as ex:
                    print(ex)
                    continue

            for index in range(num_files):
                with open(f'{output_dir}/sub_dataset_{index}.json', 'a') as output_file:
                    output_file.write("\n]")


def json2csv_1(protocol_type, slidding_window_directory):
    slidding_window_jsons = [file for file in os.listdir(slidding_window_directory) if
                             file.endswith(f"_slidding_window_{protocol_type}.pcap")]
    dataset_output = f'dataset_{protocol_type}.json'
    first_item = True
    for file in slidding_window_jsons:
        with open(file, 'rb') as file:
            # print(input_file_path)
            # first_items = []
            # size = 50000
            total_datasets = 0
            jsonobj = ijson.items(file, 'item', use_float=True)
            """for _ in range(60):  # total_datasets):
                # with open(f'sub_dataset_{index}.json', 'a') as output_file:
                # output_file.write('[\n')
                first_items.append(True)"""
            # num_files = 60
            jsonobj = ijson.items(file, 'item', use_float=True)
            counter = 0
            for item in jsonobj:
                counter += 1
                #index = counter
                # for index, item in jsonobj:
                #print(f'index: {index}')
                try:
                    with open(dataset_output, 'a') as output_file:
                        if not first_item:
                            output_file.write(",\n")
                        else:  # first_items[index]:
                            output_file.write('[\n')
                            first_item = False
                        json_string = json.dumps(item, indent=4)
                        # json_string = json_string[1:-1]
                        output_file.write(json_string)
                except Exception as ex:
                    print(ex)
                    continue

    with open(dataset_output, "r") as json_file:
        data = json.load(json_file)

    formatted_data = []

    # Loop through each record in the list
    for record in data:
        formatted_record = {
            "Context": record["Context"],
            "Question": record["Question"],
            "Answer": record["Answer"]
        }
        formatted_data.append(formatted_record)

    # Create a DataFrame from the formatted data
    df = pd.DataFrame(formatted_data)

    # Perform your operations on the DataFrame
    df = df.fillna("")

    text_col = []
    for _, row in df.iterrows():
        prompt = (f"Provided below is a question detailing key attributes of a {protocol_type} packet. Preceding this "
                  f"question is a series of {protocol_type} packets, which together form the context. Please continue "
                  f"the sequence by providing the subsequent {protocol_type} packet that follows the question, "
                  f"serving as the answer. \n\n")
        question = str(row["Question"])
        context = str(row["Context"])
        answer = str(row["Answer"])

        text = prompt + "### Context:\n" + context + "\n### Question\n" + question + "\n### Answer:\n" + answer

        text_col.append(text)

    df.loc[:, "text"] = text_col

    # Print or save the DataFrame as needed
    print(df.head())
    df.to_csv(f"{protocol_type}_dataset.csv", index=False)


def analyze_pcap_files(directory):
    input_directory = directory
    output_directory = input_directory

    # Get a list of pcap files in the input directory
    pcap_files = [file for file in os.listdir(input_directory) if file.endswith(".pcap")]

    for pcap_file in pcap_files:
        # Full path of input pcap file
        check = 1
        if all(protocol not in pcap_file for protocol in protocol_vector):
            input_pcap_path = os.path.join(input_directory, pcap_file)

            # Command to run for each pcap file
            command = f"tshark -r \"{input_pcap_path}\" -c 1 -T fields -e tcp.flags"

            # Run the command and capture the output
            result = subprocess.run(command, shell=True, capture_output=True, text=True)

            # Check if the command was successful
            if result.returncode == 0:
                # Access the output as a string
                output_string = result.stdout.strip()

                if output_string:
                    # If the output string is not empty, modify the file name
                    new_file_name = f"{os.path.splitext(pcap_file)[0]}_{protocol_vector[0]}.pcap"
                    print(f"Output String is not empty for {pcap_file}. New file name:", new_file_name)
                    check = 0

                    # Move the original pcap file to the output directory
                    output_pcap_path = os.path.join(output_directory, new_file_name)
                    os.rename(input_pcap_path, output_pcap_path)
            else:
                # Handle the case where the command failed
                print(f"Error for {pcap_file}:", result.stderr)

            if check:
                command = f"tshark -r \"{input_pcap_path}\" -c 1 -T fields -e mavlink_proto.magic"
                # Run the command and capture the output
                result = subprocess.run(command, shell=True, capture_output=True, text=True)
                if result.returncode == 0:
                    # Access the output as a string
                    output_string = result.stdout.strip()
                    if output_string:
                        # If the output string is not empty, modify the file name
                        new_file_name = f"{os.path.splitext(pcap_file)[0]}_{protocol_vector[2]}.pcap"
                        print(f"Output String is not empty for {pcap_file}. New file name:", new_file_name)
                        check = 0

                        # Move the original pcap file to the output directory
                        output_pcap_path = os.path.join(output_directory, new_file_name)
                        os.rename(input_pcap_path, output_pcap_path)
                else:
                    # Handle the case where the command failed
                    print(f"Error for {pcap_file}:", result.stderr)
            if check:
                command = f"tshark -r \"{input_pcap_path}\" -c 1 -T fields -e udp.srcport"
                # Run the command and capture the output
                result = subprocess.run(command, shell=True, capture_output=True, text=True)
                if result.returncode == 0:
                    # Access the output as a string
                    output_string = result.stdout.strip()
                    if output_string:
                        # If the output string is not empty, modify the file name
                        new_file_name = f"{os.path.splitext(pcap_file)[0]}_{protocol_vector[1]}.pcap"
                        print(f"Output String is not empty for {pcap_file}. New file name:", new_file_name)

                        # Move the original pcap file to the output directory
                        output_pcap_path = os.path.join(output_directory, new_file_name)
                        os.rename(input_pcap_path, output_pcap_path)
                    else:
                        # If the output string is empty, keep the original file name
                        print(f"Output String is empty for {pcap_file}. File name remains unchanged:", pcap_file)
                else:
                    # Handle the case where the command failed
                    print(f"Error for {pcap_file}:", result.stderr)


def make_dir(directory_path):
    # Creates a directory if it does not already exist.
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)


class CaseInsensitiveChoicesAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, values.lower())


if __name__ == "__main__":
    # Processes PCAP files, analyzes them, converts to JSON, filters based on protocol, applies a sliding window,
    # optionally divides the dataset, and converts to CSV.

    # Parses command-line arguments for protocol type, context size, input directory, and dataset size.

    # Define command-line arguments
    parser = argparse.ArgumentParser(description="Converts pcaps into a fine-tuning dataset for machine learning.",
                                     epilog="Example: python your_script.py --type udp --context 4 --input "
                                            "/path/to/pcaps --dataset 1000")
    parser.add_argument("--type", "-T", type=str, choices=["udp", "U", "tcp", "T", "mavlink", "M", "all"],
                        default="all", action=CaseInsensitiveChoicesAction, help="Protocol specification.")
    parser.add_argument("--context", '-C', type=int, default=4, help="Size of context window for dataset")
    parser.add_argument("--input", '-I', type=str, default=".", help="Path to directory with pcap files")
    parser.add_argument("--dataset", '-D', type=int, default="0",
                        help="Size of each dataset, default is all items in one dataset")

    # Process based on the provided command-line arguments
    args = parser.parse_args()
    desired_type = args.type
    context_size = args.context
    path2pcaps = args.input
    dataset_size = args.dataset

    # Initialize directory and file path configuration
    directory_protocol = ["./jsons/Tcp_jsons", "./jsons/Udp_jsons", "./jsons/Mavlink_jsons"]
    protocol_vector = ["tcp", "udp", "mavlink"]
    output_directory_filtered = ["output_filtered_tcp", "output_filtered_udp", "output_filtered_mavlink"]
    output_slidding_window = ["slidding_window_tcp", "slidding_window_udp", "slidding_window_mavlink"]
    jsons_output_directory = "jsons"
    current_path = os.path.abspath(__file__)
    processed_pcaps_location = 'processed_pcaps'

    # Process the pcaps and convert them to json format
    # process_pcaps(path2pcaps)
    # analyze_pcap_files(processed_pcaps_location)
    pcaps2jsons(jsons_output_directory)

    # For a specific protocol
    for index, item in enumerate(protocol_vector):
        if desired_type in item:
            filtered_jsons(jsons_output_directory, output_directory_filtered[index], context_size, index)
            create_sliding_window_for_directory(directory_protocol[index], output_slidding_window[index], context_size + 2,
                                                protocol_vector[index])
            # if dataset size is specified
            if dataset_size != 0:
                divide(dataset_size, protocol_vector[index], output_slidding_window[index])
                process_files_json2csv(protocol_vector[index])
            # if dataset size is not specified
            else:
                json2csv_1(protocol_vector[index], output_slidding_window[index])

    # For all the given protocols
    if desired_type == 'all':
        filtered_jsons_all(jsons_output_directory, context_size)
        for index in range(len(directory_protocol)):
            create_sliding_window_for_directory(directory_protocol[index], output_slidding_window[index], context_size + 2,
                                                protocol_vector[index])
            # if dataset size is specified
            if dataset_size != 0:
                divide(dataset_size, protocol_vector[index], output_slidding_window[index])
                process_files_json2csv(protocol_vector[index])
            # if dataset size is not specified
            else:
                json2csv_1(protocol_vector[index], output_slidding_window[index])
