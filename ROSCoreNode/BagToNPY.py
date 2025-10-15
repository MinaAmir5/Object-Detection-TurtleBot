import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler


# Path to your ROS bag file
bag_file = "/home/mina/Music/merged2.bag"

# Topics to extract
error_topic = "/Encoder"       # First topic
obstacle_topic = "/trash" # Second topic
cmd_vel_topic = "/cmd_vel"             # Third topic

# Arrays to store features and outputs
features = []
outputs = []
timeStamp = []

# Open the bag file
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == error_topic:
            # Extract x and y as the first and second features
            feature_error_x = msg.x #distance
            feature_error_y = msg.y #angle
        elif topic == obstacle_topic:
            # Extract x and theta as the third and fourth features
            feature_obstacle_x = msg.data[0]  #distance
            feature_obstacle_theta = msg.data[1] #angle
        elif topic == cmd_vel_topic:
            # Extract linear.x and angular.z as outputs
            output_linear_x = msg.linear.x
            output_angular_z = msg.angular.z
            time = t

            # Combine features and outputs when all are available
        if (
                "feature_error_x" in locals()
                and "feature_error_y" in locals()
                and "feature_obstacle_x" in locals()
                and "feature_obstacle_theta" in locals()
                and "output_linear_x" in locals()
                and "output_angular_z" in locals()
        ):
            # Append to features and outputs
            features.append([feature_error_x, feature_error_y, feature_obstacle_x, feature_obstacle_theta])
            outputs.append([output_linear_x, output_angular_z])
            timeStamp.append(time)

# Convert lists to NumPy arrays
features = np.array(features)
outputs = np.array(outputs)
timeStamp = np.array(timeStamp)

# Print summary
#print(f"Extracted {features.shape[0]} samples with {features.shape[1]} features.")
#print(f"Extracted {outputs.shape[0]} samples with {outputs.shape[1]} outputs.")

# Save data for later use
np.save("features.npy", features)
np.save("outputs.npy", outputs)
np.save("timeStamp.npy", timeStamp)