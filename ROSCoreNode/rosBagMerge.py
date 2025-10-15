import rosbag

# Input bag files
bag1_path = "/home/mina/Music/dataset7.bag"
bag2_path = "/home/mina/Music/dataset11.bag"
output_bag_path = "/home/mina/Music/merged2.bag"

# Create a new bag file for writing
with rosbag.Bag(output_bag_path, 'w') as output_bag:
    # Read and write messages from the first bag
    with rosbag.Bag(bag1_path, 'r') as bag1:
        for topic, msg, t in bag1.read_messages():
            output_bag.write(topic, msg, t)

    # Read and write messages from the second bag
    with rosbag.Bag(bag2_path, 'r') as bag2:
        for topic, msg, t in bag2.read_messages():
            output_bag.write(topic, msg, t)

print(f"Merged bag file created: {output_bag_path}")
