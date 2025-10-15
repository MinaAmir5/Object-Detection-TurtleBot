import os
import shutil
import random

# Paths
batches_dir = "/home/mina/Documents/PlatformIO/Projects/test2/TACO/data"  # Directory containing batch folders (e.g., batch_1, batch_2)
output_dir = "/home/mina/Documents/PlatformIO/Projects/test2/TACO/yolo/dataset"  # Directory for train, val, and test splits
split_ratios = {"train": 0.8, "val": 0.1, "test": 0.1}  # Train/Validation/Test split ratios

# Create output directories
for split in split_ratios.keys():
    os.makedirs(os.path.join(output_dir, split, "images"), exist_ok=True)
    os.makedirs(os.path.join(output_dir, split, "labels"), exist_ok=True)

# Collect all image and label file paths
image_paths = []
label_paths = []

for batch_folder in os.listdir(batches_dir):
    batch_path = os.path.join(batches_dir, batch_folder)
    if not os.path.isdir(batch_path):
        continue
    for file in os.listdir(batch_path):
        if file.endswith(".jpg"):  # Assuming image files are .jpg
            image_paths.append(os.path.join(batch_path, file))
            label_paths.append(
                os.path.join("yolo/labels", batch_folder, f"{os.path.splitext(file)[0]}.txt")
            )

# Shuffle the dataset
dataset = list(zip(image_paths, label_paths))
random.shuffle(dataset)

# Split the dataset
total_images = len(dataset)
train_split = int(split_ratios["train"] * total_images)
val_split = train_split + int(split_ratios["val"] * total_images)

splits = {
    "train": dataset[:train_split],
    "val": dataset[train_split:val_split],
    "test": dataset[val_split:],
}

# Copy files to corresponding split directories
for split, files in splits.items():
    for img_path, lbl_path in files:
        # Copy images
        shutil.copy(img_path, os.path.join(output_dir, split, "images", os.path.basename(img_path)))

        # Copy labels (only if the label file exists)
        if os.path.exists(lbl_path):
            shutil.copy(lbl_path, os.path.join(output_dir, split, "labels", os.path.basename(lbl_path)))

print("Dataset splitting complete!")
