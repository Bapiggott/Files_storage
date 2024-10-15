import requests

def get_latest_detection():
    """
    Retrieves the latest detection result from the YOLO server.
    """
    yolo_server_url = "http://localhost:5000/detections/latest"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            detection_result = response.json()
            print("Latest Detection:")
            print(detection_result)
            return detection_result
        elif response.status_code == 404:
            print("No detection results available.")
            return None
        else:
            print(f"Error: Received status code {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return None

def get_recent_detections():
    """
    Retrieves the last 5 detection results from the YOLO server.
    """
    yolo_server_url = "http://localhost:5000/detections/recent"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            detection_results = response.json()
            print(f"detection_results {detection_results}")
            result = []
            for sublist in detection_results:
                for item in sublist:
                    search_term = 'airplane'
                    if item['name'] == search_term:
                        result.append(item)

            # Print the results
            for entry in result:
                print(entry)
            print("~~~~~~~~")
            print(f"Last {len(detection_results)} Detections:")
            for idx, detection in enumerate(detection_results, start=1):
                print(f"\nDetection {idx}:")
                print(detection)
            return detection_results
        elif response.status_code == 404:
            print("No detection results available.")
            return []
        else:
            print(f"Error: Received status code {response.status_code}")
            return []
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return []

if __name__ == "__main__":
    # Get the latest detection
    latest_detection = get_latest_detection()
    if latest_detection:
        # Process the latest detection as needed
        pass

    # Get the last 5 detections
    recent_detections = get_recent_detections()
    if recent_detections:
        # Process the list of recent detections as needed
        pass

