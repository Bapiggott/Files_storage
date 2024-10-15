from flask import Flask, request, jsonify, send_from_directory, render_template_string
import cv2
import numpy as np
import torch
import io
import os
import time
from PIL import Image
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
from collections import deque
import threading

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
DEPTH_FOLDER = 'depth_outputs'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['DEPTH_FOLDER'] = DEPTH_FOLDER

# Set the number of latest detections to store
N = 5
latest_detections = deque(maxlen=N)
detections_lock = threading.Lock()

# Load the YOLO model
yolo_model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Adjust as needed

# Load the depth estimation model
depth_model_name = "depth-anything/Depth-Anything-V2-Small-hf"
image_processor = AutoImageProcessor.from_pretrained(depth_model_name)
depth_model = AutoModelForDepthEstimation.from_pretrained(depth_model_name)

if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

if not os.path.exists(DEPTH_FOLDER):
    os.makedirs(DEPTH_FOLDER)

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>YOLO and Depth Estimation Server</title>
    <script>
        function refreshImage() {
            var timestamp = new Date().getTime();
            document.getElementById('latest-image').src = '/image/latest?' + timestamp;
            document.getElementById('latest-depth-image').src = '/depth/latest?' + timestamp;
        }

        setInterval(refreshImage, 5000);
    </script>
</head>
<body>
    <h1>Welcome to YOLO and Depth Estimation Server</h1>
    <form action="/detect" method="post" enctype="multipart/form-data">
        <input type="file" name="image" required>
        <input type="submit" value="Upload Image">
    </form>
    <h2>Latest Uploaded Image:</h2>
    <img id="latest-image" src="/image/latest" alt="Latest Image" style="max-width: 100%;">
    
    <h2>Latest Depth Estimation Image:</h2>
    <img id="latest-depth-image" src="/depth/latest" alt="Depth Estimation Image" style="max-width: 100%;">
    
    <h3>Depth Estimation Time: {{ depth_time }} seconds</h3>
</body>
</html>
'''

@app.route('/')
def home():
    return render_template_string(HTML_TEMPLATE, depth_time="Not available yet")


@app.route('/detect', methods=['POST'])
def run_yolo_and_depth_estimation():
    try:
        if 'image' not in request.files:
            return 'No file part', 400

        file = request.files['image']
        if file.filename == '':
            return 'No selected file', 400

        # Process the image
        img_bytes = file.read()
        img_array = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

        # Perform YOLO detection
        yolo_results = yolo_model(img)
        detections = yolo_results.pandas().xyxy[0].to_dict(orient="records")

        # Save the image to the uploads folder for the latest image endpoint
        image_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        with open(image_path, 'wb') as f:
            f.write(img_bytes)

        # Save the detections to the shared deque
        with detections_lock:
            latest_detections.append(detections)  # Append new detections

        # Now, perform depth estimation
        pil_img = Image.open(io.BytesIO(img_bytes))
        inputs = image_processor(images=pil_img, return_tensors="pt")

        # Start timing the depth estimation
        start_time = time.time()

        with torch.no_grad():
            depth_outputs = depth_model(**inputs)
            predicted_depth = depth_outputs.predicted_depth

        # Interpolate the depth to match the original image size
        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=pil_img.size[::-1],
            mode="bicubic",
            align_corners=False
        )

        depth_array = prediction.squeeze().cpu().numpy()

        # Create and save the depth image
        formatted_depth = (depth_array * 255 / np.max(depth_array)).astype("uint8")
        depth_img = Image.fromarray(formatted_depth)
        depth_image_path = os.path.join(app.config['DEPTH_FOLDER'], f"depth_{file.filename}")
        depth_img.save(depth_image_path)

        # End timing the depth estimation
        depth_time = round(time.time() - start_time, 2)

        return render_template_string(HTML_TEMPLATE, depth_time=depth_time)

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


@app.route('/depth/latest')
def latest_depth_image():
    files = os.listdir(app.config['DEPTH_FOLDER'])
    if files:
        latest_file = max(files, key=lambda x: os.path.getctime(os.path.join(app.config['DEPTH_FOLDER'], x)))
        return send_from_directory(app.config['DEPTH_FOLDER'], latest_file)
    return '', 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

