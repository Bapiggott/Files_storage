"""import pyshark
import os
import concurrent.futures

def is_first_packet_tcp(file_path):
    try:
        with pyshark.FileCapture(file_path, only_summaries=True) as capture:
            print("trying")
            first_packet = next(iter(capture))
            return any(layer.layer_name.upper() == 'TCP' for layer in first_packet.layers)
    except StopIteration:
        # No packets in the pcap file
        return False
    except Exception as e:
        print(f"Error processing file {file_path}: {e}")
        return False

def rename_file_if_tcp(file_path):
    if is_first_packet_tcp(file_path):
        dir_name, file_name = os.path.split(file_path)
        new_file_name = f"tcp_{file_name}"
        new_file_path = os.path.join(dir_name, new_file_name)
        os.rename(file_path, new_file_path)
        print(f"Renamed '{file_path}' to '{new_file_path}'")

def process_files(pcap_directory):
    pcap_paths = [os.path.join(pcap_directory, f) for f in os.listdir(pcap_directory) if f.endswith('.pcap')]

    with concurrent.futures.ProcessPoolExecutor() as executor:
        executor.map(rename_file_if_tcp, pcap_paths)

# Directory containing pcap files - replace with the actual directory path
pcap_directory = '.'

process_files(pcap_directory)"""
import pyshark
import os
import concurrent.futures

def is_first_packet_tcp(file_path):
    try:
        with pyshark.FileCapture(file_path, only_summaries=True, keep_packets=False) as capture:
            print("trying")
            capture.set_debug()
            for packet in capture:
                return any(layer.layer_name.upper() == 'TCP' for layer in packet.layers)
    except Exception as e:
        print(f"Error processing file {file_path}: {e}")
        return False

def rename_file_if_tcp(file_path):
    if is_first_packet_tcp(file_path):
        dir_name, file_name = os.path.split(file_path)
        new_file_name = f"tcp_{file_name}"
        new_file_path = os.path.join(dir_name, new_file_name)
        os.rename(file_path, new_file_path)
        print(f"Renamed '{file_path}' to '{new_file_path}'")

def process_file(file_path):
    # if os.path.getsize(file_path) < 10000000:  # 10MB size limit, adjust as needed
    rename_file_if_tcp(file_path)


def process_files(pcap_directory):
    pcap_paths = [os.path.join(pcap_directory, f) for f in os.listdir(pcap_directory) if f.endswith('.pcap')]

    with concurrent.futures.ThreadPoolExecutor() as executor:
        for pcap_path in pcap_paths:
            executor.submit(process_file, pcap_path)

# Directory containing pcap files - replace with the actual directory path
pcap_directory = '.'

process_files(pcap_directory)

