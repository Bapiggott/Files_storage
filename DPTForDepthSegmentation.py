from transformers import AutoImageProcessor, DPTForDepthEstimation, pipeline
import torch
import numpy as np
from PIL import Image
import requests
import matplotlib.pyplot as plt

# Load the image
url = "http://images.cocodataset.org/val2017/000000039769.jpg"
image = Image.open(requests.get(url, stream=True).raw)

# 1. Semantic Segmentation

# Load a semantic segmentation model (SegFormer)
semantic_segmentation = pipeline("image-segmentation", model="nvidia/segformer-b1-finetuned-cityscapes-1024-1024")

# Perform semantic segmentation
segmentation_results = semantic_segmentation(image)

# Visualize the segmentation results by overlaying the segmentation masks
plt.figure(figsize=(10, 5))

# Plot the original image
plt.subplot(1, 2, 1)
plt.title("Original Image")
plt.imshow(image)

# Plot the segmentation mask
plt.subplot(1, 2, 2)
plt.title("Segmentation Mask")
for result in segmentation_results:
    mask = np.array(result["mask"])
    plt.imshow(mask, alpha=0.5)  # Overlay the mask with transparency
plt.axis("off")

# Show the combined segmentation results
plt.show()

# Optionally, save the segmentation results
for idx, result in enumerate(segmentation_results):
    result["mask"].save(f"segmentation_mask_{result['label']}.png")

# 2. Depth Estimation (after segmentation)

# Load the image processor and the depth estimation model
image_processor = AutoImageProcessor.from_pretrained("Intel/dpt-large")
depth_model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")

# Prepare the image for the depth model
inputs = image_processor(images=image, return_tensors="pt")

# Perform inference to get the depth prediction
with torch.no_grad():
    outputs = depth_model(**inputs)
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

# Save the depth image for visualization
formatted_depth = (depth_array * 255 / np.max(depth_array)).astype("uint8")
depth_image = Image.fromarray(formatted_depth)
depth_image.save("depth_estimation_output.png")

# Visualize the depth map and segmentation together
plt.figure(figsize=(10, 5))

# Plot the segmentation mask
plt.subplot(1, 2, 1)
plt.title("Segmentation Mask")
for result in segmentation_results:
    mask = np.array(result["mask"])
    plt.imshow(mask, alpha=0.5)
plt.axis("off")

# Plot the depth map
plt.subplot(1, 2, 2)
plt.title("Depth Estimation")
plt.imshow(formatted_depth, cmap="plasma")  # Show depth map with a colormap
plt.axis("off")

plt.show()

# Optionally, print the shape of the depth array
print("Depth array shape:", depth_array.shape)

