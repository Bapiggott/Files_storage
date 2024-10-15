"""from transformers import AutoImageProcessor, DPTForDepthEstimation

import torch

import numpy as np

from PIL import Image

import requests
from transformers import DPTForSemanticSegmentation


url = "http://images.cocodataset.org/val2017/000000039769.jpg"

image = Image.open(requests.get(url, stream=True).raw)
image.save("depth_estimation_input.png")

image_processor = AutoImageProcessor.from_pretrained("Intel/dpt-large")

model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")

# prepare image for the model

inputs = image_processor(images=image, return_tensors="pt")

with torch.no_grad():

    outputs = model(**inputs)

    predicted_depth = outputs.predicted_depth

# interpolate to original size

prediction = torch.nn.functional.interpolate(

    predicted_depth.unsqueeze(1),

    size=image.size[::-1],

    mode="bicubic",

    align_corners=False,

)

# visualize the prediction
output = prediction.squeeze().cpu().numpy()

formatted = (output * 255 / np.max(output)).astype("uint8")

depth = Image.fromarray(formatted)

# save the depth estimation image
depth.save("depth_estimation_output.png")
print("--"*6)
image_processor = AutoImageProcessor.from_pretrained("Intel/dpt-large-ade")

model = DPTForSemanticSegmentation.from_pretrained("Intel/dpt-large-ade")

inputs = image_processor(images=image, return_tensors="pt")

outputs = model(**inputs)

logits = outputs.logits"""
from transformers import AutoImageProcessor, DPTForDepthEstimation
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
import torch
import numpy as np
from PIL import Image
import requests

# Load the image
url = "http://images.cocodataset.org/val2017/000000039769.jpg"
image = Image.open(requests.get(url, stream=True).raw)
model_name = "depth-anything/Depth-Anything-V2-Small-hf"
#model_name = "Intel/dpt-large"
# Load the image processor and the model
#image_processor = AutoImageProcessor.from_pretrained("Intel/dpt-large")
#model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")
image_processor = AutoImageProcessor.from_pretrained(model_name)

model = AutoModelForDepthEstimation.from_pretrained(model_name)


# Prepare image for the model
inputs = image_processor(images=image, return_tensors="pt")

# Perform inference to get the depth prediction
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

# Convert the depth map to a multi-dimensional numpy array
depth_array = prediction.squeeze().cpu().numpy()

# Create an array of [x, y, depth] for each pixel
height, width = depth_array.shape
depth_with_coordinates = []

for y in range(height):
    for x in range(width):
        depth_with_coordinates.append([x, y, depth_array[y, x]])

# Convert to a NumPy array
depth_with_coordinates = np.array(depth_with_coordinates)

# Save the array to a .txt file in human-readable form
np.savetxt(f"depth_with_coordinates.txt", depth_with_coordinates, fmt='%.4f')

# Optionally, save it to a .npy file
np.save("depth_with_coordinates.npy", depth_with_coordinates)

# Visualize the depth map and save it as an image (optional)
formatted = (depth_array * 255 / np.max(depth_array)).astype("uint8")
depth_image = Image.fromarray(formatted)
depth_image.save(f"depth_estimation_output.png")

# Optionally, print the shape of the array
print("Depth array with coordinates shape:", depth_with_coordinates.shape)

