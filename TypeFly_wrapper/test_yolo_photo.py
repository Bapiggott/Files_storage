import requests
import os
import time
from datetime import datetime

def get_latest_image():
    """
    Retrieves the latest image from the YOLO server and saves it with a timestamp.
    """
    yolo_server_url = "http://localhost:5000/image/latest"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            # Get the current timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"latest_image_{timestamp}.jpg"
            
            # Save the image to the current directory
            with open(filename, 'wb') as f:
                f.write(response.content)
            
            print(f"Image saved as {filename}")
            return filename
        else:
            print(f"Failed to get image: Status code {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return None

def display_image(filename):
    """
    Displays the image in the terminal using ASCII characters (optional).
    Note: This requires the 'PIL' and 'numpy' libraries.
    """
    try:
        from PIL import Image
        import numpy as np

        img = Image.open(filename)
        img.thumbnail((80, 40))  # Resize for terminal display
        img_array = np.array(img)

        # Convert to grayscale
        img_gray = img.convert('L')
        img_array = np.array(img_gray)

        # Define ASCII characters
        ascii_chars = [' ', '.', ':', '-', '=', '+', '*', '#', '%', '@']
        ascii_str = ''
        for pixel_row in img_array:
            for pixel in pixel_row:
                ascii_str += ascii_chars[pixel // 25]
            ascii_str += '\n'

        print(ascii_str)
    except ImportError:
        print("PIL and numpy are required for displaying the image in the terminal.")
        print("You can install them using 'pip install Pillow numpy'.")

if __name__ == "__main__":
    filename = get_latest_image()
    if filename:
        # Optionally display the image in the terminal
        display_image(filename)

