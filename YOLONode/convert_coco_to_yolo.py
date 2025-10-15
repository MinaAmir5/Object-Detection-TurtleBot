import os
import json
from tqdm import tqdm

# Paths
coco_json_path = "/home/mina/Documents/PlatformIO/Projects/test2/TACO/data/annotations.json"  # Path to COCO annotations file
batches_dir = "/home/mina/Documents/PlatformIO/Projects/test2/TACO/data"                     # Directory containing batch directories (e.g., batch_1, batch_2)
output_dir = "/home/mina/Documents/PlatformIO/Projects/test2/TACO/yolo"
labels_dir = os.path.join(output_dir, "labels")

# Create directories for YOLO format
os.makedirs(labels_dir, exist_ok=True)

# Load COCO JSON
with open(coco_json_path, "r") as f:
    coco_data = json.load(f)

# Get categories
categories = {cat["id"]: cat["name"] for cat in coco_data["categories"]}

# Normalize function
def convert_bbox_coco_to_yolo(image_width, image_height, bbox):
    x, y, w, h = bbox
    x_center = (x + w / 2) / image_width
    y_center = (y + h / 2) / image_height
    width = w / image_width
    height = h / image_height
    return x_center, y_center, width, height

# Process annotations
images = {img["id"]: img for img in coco_data["images"]}
annotations = coco_data["annotations"]

for ann in tqdm(annotations, desc="Converting annotations"):
    image_id = ann["image_id"]
    image = images[image_id]
    image_filename = image["file_name"]  # Example: "batch_1/000001.jpg"
    image_width = image["width"]
    image_height = image["height"]

    # Convert bounding box
    bbox = ann["bbox"]
    x_center, y_center, width, height = convert_bbox_coco_to_yolo(image_width, image_height, bbox)

    # Get category ID
    category_id = ann["category_id"]
    class_id = list(categories.keys()).index(category_id)

    # Ensure the corresponding batch folder exists in the labels directory
    batch_folder = os.path.dirname(image_filename)  # Extract the batch folder (e.g., "batch_1")
    label_batch_folder = os.path.join(labels_dir, batch_folder)
    os.makedirs(label_batch_folder, exist_ok=True)

    # Write YOLO annotation file
    label_file = os.path.join(label_batch_folder, f"{os.path.splitext(os.path.basename(image_filename))[0]}.txt")
    with open(label_file, "a") as f:
        f.write(f"{class_id} {x_center} {y_center} {width} {height}\n")

print("COCO to YOLO conversion complete!")
