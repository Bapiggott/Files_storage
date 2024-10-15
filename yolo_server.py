from flask import Flask, request, jsonify, send_from_directory, render_template_string
import cv2
import numpy as np
import torch
import os

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'  # Folder to save uploaded images
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

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
        
        # Save the uploaded image
        image_path = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        file.save(image_path)

        # Read and process the image with YOLO
        img = cv2.imread(image_path)
        results = model(img)
        detections = results.pandas().xyxy[0].to_dict(orient="records")

        return jsonify(detections)

    except Exception as e:
        return str(e), 400

@app.route('/image/latest')
def latest_image():
    files = os.listdir(app.config['UPLOAD_FOLDER'])
    if files:
        latest_file = max(files, key=lambda x: os.path.getctime(os.path.join(app.config['UPLOAD_FOLDER'], x)))
        return send_from_directory(app.config['UPLOAD_FOLDER'], latest_file)
    return '', 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
