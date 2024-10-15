from transformers import AutoImageProcessor, DPTForDepthEstimation
import torch
import numpy as np
from PIL import Image
import requests

# Load the image
url = "http://images.cocodataset.org/val2017/000000039769.jpg"
image = Image.open(requests.get(url, stream=True).raw)

# Load the DPT model fine-tuned on the KITTI dataset (with depth in meters)
image_processor = AutoImageProcessor.from_pretrained("Intel/dpt-large-kitti")
model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large-kitti")

# Prepare the image for the model
inputs = image_processor(images=image, return_tensors="pt")

# Perform inference to get the depth prediction (in meters)
with torch.no_grad():
    outputs = model(**inputs)
    predicted_depth = outputs.predicted_depth

# Interpolate the predicted depth to the original image size
prediction = torch.nn.functional.interpolate(
    predicted_depth.unsqueeze(1),  # Add channel dimension
    size=image.size[::-1],         # Resize to original image size
    mode="bicubic",                # Use bicubic interpolation
    align_corners=False,
)

# Convert the depth map to a multi-dimensional numpy array (in meters)
depth_array_in_meters = prediction.squeeze().cpu().numpy()

# Create an array of [x, y, depth] for each pixel
height, width = depth_array_in_meters.shape
depth_with_coordinates = []

for y in range(height):
    for x in range(width):
        depth_with_coordinates.append([x, y, depth_array_in_meters[y, x]])

# Convert to a NumPy array
depth_with_coordinates = np.array(depth_with_coordinates)

# Save the array to a .txt file (readable)
np.savetxt("depth_with_coordinates_kitti.txt", depth_with_coordinates, fmt='%.4f')

# Optionally, save it to a .npy file
np.save("depth_with_coordinates_kitti.npy", depth_with_coordinates)

# Visualize the depth map and save it as an image (optional)
formatted = (depth_array_in_meters * 255 / np.max(depth_array_in_meters)).astype("uint8")
depth_image = Image.fromarray(formatted)
depth_image.save("depth_estimation_output_kitti.png")

# Optionally, print the shape of the array
print("Depth array with coordinates (in meters) shape:", depth_with_coordinates.shape)

