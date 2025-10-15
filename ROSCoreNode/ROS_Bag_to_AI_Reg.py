import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler


# Read ROS bag data from three topics
def read_rosbag(dataset, topics):
    bag = rosbag.Bag(dataset)
    data = {topic: [] for topic in topics}
    time_stamps = {topic: [] for topic in topics}

    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == topics[0]:  # First topic
            time_stamps[topic].append(t.to_sec())
            data[topic].append([msg.linear.x, msg.angular.z])
        elif topic == topics[1]:  # Second topic
            time_stamps[topic].append(t.to_sec())
            data[topic].append([msg.x, msg.y])
        elif topic == topics[2]:  # Third topic
            time_stamps[topic].append(t.to_sec())
            data[topic].append([msg.data[0], msg.data[1]])

    bag.close()

    # Convert to numpy arrays
    for topic in topics:
        data[topic] = np.array(data[topic])
        time_stamps[topic] = np.array(time_stamps[topic])

    return time_stamps, data


# Normalize data for each topic
def normalize_data(data):
    normalized_data = {}
    scalers = {}
    for topic, values in data.items():
        scaler = MinMaxScaler()
        normalized_data[topic] = scaler.fit_transform(values)
        scalers[topic] = scaler
    return normalized_data, scalers


# Visualize data for each topic
def visualize_data(time_stamps, data, normalized_data):
    num_topics = len(data)
    plt.figure(figsize=(15, 5 * num_topics))

    for i, (topic, values) in enumerate(data.items()):
        plt.subplot(num_topics, 2, i * 2 + 1)
        plt.plot(time_stamps[topic], values[:, 0], label="Var 1")
        plt.plot(time_stamps[topic], values[:, 1], label="Var 2")
        plt.title(f"Raw Data for {topic}")
        plt.xlabel("Time (s)")
        plt.ylabel("Values")
        plt.legend()

        plt.subplot(num_topics, 2, i * 2 + 2)
        plt.plot(time_stamps[topic], normalized_data[topic][:, 0], label="Normalized Var 1")
        plt.plot(time_stamps[topic], normalized_data[topic][:, 1], label="Normalized Var 2")
        plt.title(f"Normalized Data for {topic}")
        plt.xlabel("Time (s)")
        plt.ylabel("Normalized Values")
        plt.legend()

    plt.tight_layout()
    plt.show()


# Main function
def main():
    bag_path = "/home/mina/Music/dataset0.bag"  # Replace with your bag file path
    topics = ["/cmd_vel", "/Encoder", "/trash"]  # Replace with your topics

    # Read data from ROS bag
    time_stamps, data = read_rosbag(bag_path, topics)

    # Normalize the data
    normalized_data, scalers = normalize_data(data)

    # Visualize raw and normalized data
    visualize_data(time_stamps, data, normalized_data)

    # Print normalized data shape for each topic
    for topic in topics:
        print(f"Normalized Data Shape for {topic}: {normalized_data[topic].shape}")


if __name__ == "__main__":
    main()
