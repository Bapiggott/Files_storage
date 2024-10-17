import pyshark

def extract_ports(pcap_file):
    source_ports = set()
    destination_ports = set()

    cap = pyshark.FileCapture(pcap_file)

    for packet in cap:
        try:
            source_ports.add(packet.transport_layer.srcport)
            destination_ports.add(packet.transport_layer.dstport)
        except AttributeError:
            # Skip packets that don't have source or destination ports
            pass

    return source_ports, destination_ports

def main():
    pcap_file = "capture_1.pcap"  # Replace with your actual PCAP file
    source_ports, destination_ports = extract_ports(pcap_file)

    print("Source Ports:")
    print(sorted(source_ports))

    print("\nDestination Ports:")
    print(sorted(destination_ports))

if __name__ == "__main__":
    main()

