import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2
import rosbag2_py
import numpy as np
import struct
import csv
import os
import owncloud  # Bibliothèque pour le Cloud
import time  # Pour générer des timestamps
import shutil

class LiDARRecorder(Node):
    def __init__(self):
        super().__init__('lidar_recorder')

        # Paramètres pour le bag
        self.bag_dir = "/home/mobilabls/Workspace/ros-bridge/lidar_bag"
        self.bag_filename_base = "lidar_bag"
        self.bag_index = 0
        self.index_dir = 0
        self.extension = "_0.db3"
        self.max_bag_size_mb = 10.0  # Taille maximale en MB

        # Créer le dossier si nécessaire
        if not os.path.exists(self.bag_dir):
            os.makedirs(self.bag_dir)

        self.initialize_writer()

        # 🔹 Création d'un Subscriber pour récupérer les données LiDAR
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carla/hero/lidar',
            self.lidar_callback,
            10
        )

        # 🔹 Ouverture d'un fichier CSV pour stocker les données
        self.csv_filename = "lidar_data.csv"
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["x", "y", "z", "intensity"])  # En-têtes du fichier CSV

        self.get_logger().info("✅ LiDAR Recorder Initialized")

    def initialize_writer(self):
        """ Initialise (ou réinitialise) le writer pour le bag actuel. """
        # Génère le chemin du bag courant
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
        """ Callback exécuté à chaque réception de message LiDAR """
        # Enregistrement dans le ROS2 Bag
        self.writer.write(
            '/carla/hero/lidar',
            serialize_message(msg),
            self.get_clock().now().nanoseconds
        )
        self.get_logger().info("📡 LiDAR Data Recorded in Bag")

        # Conversion et écriture dans le CSV
        point_cloud = self.convert_pointcloud2_to_numpy(msg)
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(point_cloud)
        self.get_logger().info(f"✅ LiDAR Data Saved to {self.csv_filename}")

        # Vérifier la taille et procéder à la rotation si nécessaire
        self.check_and_rotate_bag()

    def convert_pointcloud2_to_numpy(self, msg):
        """ Convertit le message ROS2 PointCloud2 en un tableau NumPy de points (x, y, z, intensité) """
        data = np.frombuffer(msg.data, dtype=np.float32)
        num_points = len(data) // 4
        return data.reshape(num_points, 4)

    def check_and_rotate_bag(self):
        """ Vérifie la taille du bag, et si elle atteint le seuil, upload, supprime et réinitialise le bag """
        bag_path1 = "/home/mobilabls/Workspace/ros-bridge/lidar_bag/lidar_bag_"
        bag_path = f"{bag_path1}{self.index_dir}"
        name_file = os.path.join(bag_path, f"{self.current_bag_filename}{self.extension}")
        print(name_file)
        
        if os.path.exists(name_file):
            bag_size = os.path.getsize(name_file) / (1024 * 1024)  # en MB
            self.get_logger().info(f"📂 Bag Size ({self.current_bag_filename}): {bag_size:.2f} MB")

            if bag_size >= 5:
                self.get_logger().info("🚀 Bag size threshold reached. Initiating upload and rotation...")
                self.upload_to_cloud(name_file)
                self.index_dir +=1
                self.rotate_bag(bag_path)
        else:
            self.get_logger().info("No bag file detected.")

    def upload_to_cloud(self, file_path):
        """ Upload du ROS2 Bag sur OwnCloud dans le dossier Group02 """
        try:
            public_link = 'https://sync.academiccloud.de/index.php/s/PvhwqYt6JxcEa1Z'
            oc = owncloud.Client.from_public_link(public_link)

            
            # 🔹 Upload du fichier dans Group02
            oc.drop_file(file_path)
            

            self.get_logger().info("✅ Bag successfully uploaded to the cloud in Group02!")

            oc.logout()

        except Exception as e:
            self.get_logger().error(f"❌ Cloud upload failed: {e}")

    def rotate_bag(self, bag_path):
        """ Ferme le bag actuel, le supprime localement, incrémente l'index et réinitialise le writer """
        try:
            if hasattr(self.writer, "close"):
                self.writer.close()
        except Exception as e:
            self.get_logger().warn(f"Warning while closing writer: {e}")

        try:
            shutil.rmtree(bag_path)  # Utilisation de shutil.rmtree pour supprimer un répertoire
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

