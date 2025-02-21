import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2
import rosbag2_py
import numpy as np
import csv
import os
import owncloud  # Library for the Cloud
import time
import shutil
import socket

def is_internet_available(timeout=2):
    """
    Returns True if we can connect to 8.8.8.8 (Google DNS)
    within 'timeout' seconds, False otherwise.
    """
    try:
        socket.setdefaulttimeout(timeout)
        with socket.create_connection(("8.8.8.8", 53), timeout=timeout):
            return True
    except OSError:
        return False

class LiDARRecorder(Node):
    def __init__(self):
        super().__init__('lidar_recorder')

        # Bag parameters
        self.bag_dir = "/home/mobilabls/Workspace/ros-bridge/lidar_bag"
        self.bag_filename_base = "lidar_bag"
        self.bag_index = 0
        self.index_dir = 0
        self.extension = "_0.db3"
        self.max_bag_size_mb = 10.0  # Maximum bag size in MB

        # Create the directory if needed
        if not os.path.exists(self.bag_dir):
            os.makedirs(self.bag_dir)

        self.initialize_writer()

        # 🔹 Create a Subscriber for LiDAR data
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.lidar_callback,
            10
        )

        # 🔹 Open a CSV file to store LiDAR data
        self.csv_filename = "lidar_data.csv"
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["x", "y", "z", "intensity"])  # CSV headers

        self.get_logger().info("✅ LiDAR Recorder Initialized")

    def initialize_writer(self):
        """ Initialize (or re-initialize) the writer for the current bag. """
        self.current_bag_filename = f"{self.bag_filename_base}_{self.bag_index}"
        self.current_bag_path = os.path.join(self.bag_dir, self.current_bag_filename)
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri=self.current_bag_path,
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            name='/carla/hero/lidar',
            type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr'
        )
        self.writer.create_topic(topic_info)

        self.get_logger().info(f"🆕 New bag initialized: {self.current_bag_filename}")

    def lidar_callback(self, msg):
        """ Callback triggered upon receiving each LiDAR message. """

        # 1) Check internet connection
        if not is_internet_available():
            self.get_logger().warn("❌ Internet connection lost, stopping recording!")
            self.stop_recording()
            return

        # 2) Record into the ROS 2 Bag
        self.writer.write(
            '/carla/hero/lidar',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
        self.get_logger().info("📡 LiDAR Data Recorded in Bag")

        # 3) Convert and write to CSV
        point_cloud = self.convert_pointcloud2_to_numpy(msg)
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(point_cloud)
        self.get_logger().info(f"✅ LiDAR Data Saved to {self.csv_filename}")

        # 4) Check bag size and rotate if needed
        self.check_and_rotate_bag()

    def stop_recording(self):
        """Stop recording: close the writer and stop the node."""
        try:
            if hasattr(self.writer, "close"):
                self.writer.close()
        except Exception as e:
            self.get_logger().warn(f"Warning while closing writer: {e}")

        self.get_logger().info("Node is stopping because internet is unavailable.")
        self.destroy_node()

    def convert_pointcloud2_to_numpy(self, msg):
        """ Convert the PointCloud2 message into a NumPy array of points (x, y, z, intensity). """
        data = np.frombuffer(msg.data, dtype=np.float32)
        num_points = len(data) // 4
        return data.reshape(num_points, 4)

    def check_and_rotate_bag(self):
        """ Check bag size, upload, remove, and re-init bag if it exceeds the threshold. """
        bag_path1 = "/home/mobilabls/Workspace/ros-bridge/lidar_bag/lidar_bag_"
        bag_path = f"{bag_path1}{self.index_dir}"
        name_file = os.path.join(bag_path, f"{self.current_bag_filename}{self.extension}")
        print(name_file)

        if os.path.exists(name_file):
            bag_size = os.path.getsize(name_file) / (1024 * 1024)
            self.get_logger().info(f"📂 Bag Size ({self.current_bag_filename}): {bag_size:.2f} MB")

            if bag_size >= 5:
                self.get_logger().info("🚀 Bag size threshold reached. Uploading and rotating...")
                self.upload_to_cloud(name_file)
                self.index_dir += 1
                self.rotate_bag(bag_path)
        else:
            self.get_logger().info("No bag file detected.")

    def upload_to_cloud(self, file_path):
        """ Upload the ROS2 Bag to OwnCloud in the Group02 folder. """
        try:
            public_link = 'https://sync.academiccloud.de/index.php/s/PvhwqYt6JxcEa1Z'
            oc = owncloud.Client.from_public_link(public_link)

            # 🔹 Upload the file to Group02
            oc.drop_file(file_path)

            self.get_logger().info("✅ Bag successfully uploaded to the cloud in Group02!")
            oc.logout()

        except Exception as e:
            self.get_logger().error(f"❌ Cloud upload failed: {e}")

    def rotate_bag(self, bag_path):
        """ Close the current bag, remove it locally, increment index, and re-init the writer. """
        try:
            if hasattr(self.writer, "close"):
                self.writer.close()
        except Exception as e:
            self.get_logger().warn(f"Warning while closing writer: {e}")

        try:
            shutil.rmtree(bag_path)
            self.get_logger().info(f"🗑️ Deleted local bag directory: {self.current_bag_filename}")
        except Exception as e:
            self.get_logger().warn(f"Could not delete local bag directory: {e}")

        self.bag_index += 1
        self.initialize_writer()

def main(args=None):
    rclpy.init(args=args)
    node = LiDARRecorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

