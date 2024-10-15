import requests

data = [
    [{'class': 4, 'confidence': 0.27853134274482727, 'name': 'airplane', 'xmax': 1916.555419921875, 'xmin': 1517.49169921875, 'ymax': 594.8165283203125, 'ymin': 505.32025146484375},
     {'class': 4, 'confidence': 0.25895795226097107, 'name': 'airplane', 'xmax': 1917.1519775390625, 'xmin': 1519.8599853515625, 'ymax': 896.4505004882812, 'ymin': 515.3161010742188}],
    [{'class': 4, 'confidence': 0.2743082642555237, 'name': 'airplne', 'xmax': 1916.931640625, 'xmin': 1517.525634765625, 'ymax': 595.0516967773438, 'ymin': 505.11981201171875},
     {'class': 4, 'confidence': 0.27092233300209045, 'name': 'airplane', 'xmax': 1917.2449951171875, 'xmin': 1519.8382568359375, 'ymax': 896.133544921875, 'ymin': 515.072509765625}],
    # Add remaining dictionaries...
]
def get_latest_detection():
    """
    Retrieves the latest detection result from the YOLO server.
    """
    yolo_server_url = "http://localhost:5000/detections/latest"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            detection_result = response.json()
            """print("Latest Detection:")
            print(detection_result)"""
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
def objectx(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                #print(item)
                if object in item["name"]:
                    objectx = ((item["xmax"] + item["xmin"]) / 2) / 1920
                    return objectx
    return  -1

def objecty(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                if object in item["name"]:
                    objecty = ((item["ymax"] + item["ymin"]) / 2) / 1200
                    return objecty
    return -1

def object_width(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                if object in item["name"]:
                    object_wid = (item["xmax"] - item["xmin"]) / 1920
                    return object_wid
    return  -1

def object_height(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                if object in item["name"]:
                    object_height = (item["ymax"] - item["ymin"]) / 1200
                    return object_height
    return  -1

object_name = 'suitcase'

from functools import partial
functions = [objectx, objecty, object_width, object_height]
# Create partial functions with the parameters fixed
objectx_partial = partial(objectx, object_name)
objecty_partial = partial(objecty, object_name)
object_width_partial = partial(object_width, object_name)
object_height_partial = partial(object_height, object_name)

# List of partial functions
partial_functions = [objectx_partial, objecty_partial, object_width_partial, object_height_partial]

# Map the functions and print the results
for func, value in zip(functions, map(lambda f: f(), partial_functions)):
    print(f"{func.__name__} returned {value}")


