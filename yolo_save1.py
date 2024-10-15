from flask import Flask, request, jsonify, send_from_directory, render_template_string
import cv2
import numpy as np
import torch
import io
import os
import json
from collections import deque
import  threading

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Set the number of latest detections to store
N = 5

# Use deque for efficient appends and pops with a fixed size
latest_detections = deque(maxlen=N)
detections_lock = threading.Lock()


# Initialize YOLO model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Adjust as needed

if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>YOLO Server</title>
    <script>
        function refreshImage() {
            // Use the current time to prevent caching
            var timestamp = new Date().getTime();
            document.getElementById('latest-image').src = '/image/latest?' + timestamp;
        }

        // Refresh image every 5 seconds (5000 milliseconds)
        setInterval(refreshImage, 5000);
    </script>
</head>
<body>
    <h1>Welcome to YOLO Server</h1>
    <form action="/detect" method="post" enctype="multipart/form-data">
        <input type="file" name="image" required>
        <input type="submit" value="Upload Image">
    </form>
    <h2>Latest Uploaded Image:</h2>
    <img id="latest-image" src="/image/latest" alt="Latest Image" style="max-width: 100%;">
</body>
</html>
'''

@app.route('/')
def home():
    return render_template_string(HTML_TEMPLATE)


@app.route('/detect', methods=['POST'])
def run_yolo():
    try:
        if 'image' not in request.files:
            return 'No file part', 400

        file = request.files['image']
        if file.filename == '':
            return 'No selected file', 400

        # Process the image directly from memory
        img_bytes = file.read()
        img_array = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        # Perform YOLO detection
        results = model(img)
        detections = results.pandas().xyxy[0].to_dict(orient="records")

        # Save the image to the uploads folder for the latest image endpoint
        image_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        with open(image_path, 'wb') as f:
            f.write(img_bytes)

        # **Save the detections to the shared deque**
        with detections_lock:
            latest_detections.append(detections)  # Append new detections

        return jsonify(detections)

    except Exception as e:
        return str(e), 400


@app.route('/detections/latest', methods=['GET'])
def get_latest_detection():
    with detections_lock:
        if latest_detections:
            latest_detection = latest_detections[-1]  # Most recent detection
            return jsonify(latest_detection)
        else:
            return jsonify([]), 404  # No detections available


@ app.route('/detections/recent', methods=['GET'])
def get_recent_detections():
    with detections_lock:
        if latest_detections:
            recent_detections = list(latest_detections)  # Convert deque to list
            return jsonify(recent_detections)
        else:
            return jsonify([]), 404  # No detections available


@app.route('/image/latest')
def latest_image():
    files = os.listdir(app.config['UPLOAD_FOLDER'])
    if files:
        latest_file = max(files, key=lambda x: os.path.getctime(os.path.join(app.config['UPLOAD_FOLDER'], x)))
        return send_from_directory(app.config['UPLOAD_FOLDER'], latest_file)
    return '', 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)