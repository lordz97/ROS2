import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2
import rosbag2_py
import numpy as np
import csv
import os
import owncloud  # Bibliothèque pour le Cloud
import time
import glob

class LiDARRecorder(Node):
    def __init__(self):
        super().__init__('lidar_recorder')

        # Paramètres pour le bag
        self.bag_dir = "/home/mobilabls/Workspace/ros-bridge/lidar_bag"
        self.bag_filename_base = "lidar_bag"
        self.bag_index = 0
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
        # Génère le chemin de base pour rosbag2
        # Note: rosbag2 va suffixer par ex. `_0.db3`
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

        self.get_logger().info(f"🆕 New bag initialized: {self.current_bag_path}")

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
        """ Vérifie la taille du (ou des) vrai(s) fichier(s) .db3 créé(s) par rosbag2 """
        actual_file = self.find_db3_file()
        if actual_file and os.path.exists(actual_file):
            bag_size = os.path.getsize(actual_file) / (1024 * 1024)  # en MB
            self.get_logger().info(f"📂 Bag Size ({os.path.basename(actual_file)}): {bag_size:.2f} MB")

            if bag_size >= self.max_bag_size_mb:
                self.get_logger().info("🚀 Bag size threshold reached. Initiating upload and rotation...")
                self.upload_to_cloud(actual_file)
                self.rotate_bag()
        else:
            self.get_logger().info("No actual bag file detected yet.")

    def find_db3_file(self):
        """
        Trouve le vrai fichier .db3 généré par rosbag2.
        Par ex., si self.current_bag_path = '/home/xxx/lidar_bag/lidar_bag_0',
        rosbag2 peut créer 'lidar_bag_0_0.db3'.
        On cherche le plus récent .db3.
        """
        pattern = self.current_bag_path + "*"
        matches = glob.glob(pattern)
        # Filtrer ceux qui se terminent par .db3
        db3_files = [m for m in matches if m.endswith(".db3")]
        if not db3_files:
            return None
        # Choisir le plus récent
        return max(db3_files, key=os.path.getmtime)

    def upload_to_cloud(self, file_path):
        """ Upload du ROS2 Bag sur OwnCloud dans le dossier Group02 en ajoutant un timestamp pour un nom unique """
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            destination_filename = f"/Group02/{self.bag_filename_base}_{timestamp}.db3"

            public_link = 'https://sync.academiccloud.de/index.php/s/PvhwqYt6JxcEa1Z'
            folder_password = '123456789'
            oc = owncloud.Client.from_public_link(public_link, folder_password=folder_password)

            oc.put_file(destination_filename, file_path)

            self.get_logger().info(f"✅ Bag uploaded as {destination_filename}!")
            oc.logout()

        except Exception as e:
            self.get_logger().error(f"❌ Cloud upload failed: {e}")

    def rotate_bag(self):
        """ Ferme le bag actuel, supprime localement, incrémente l'index et réinitialise le writer """
        try:
            if hasattr(self.writer, "close"):
                self.writer.close()
        except Exception as e:
            self.get_logger().warn(f"Warning while closing writer: {e}")

        # Supprime tous les fichiers correspondants
        pattern = self.current_bag_path + "*"
        for f in glob.glob(pattern):
            try:
                os.remove(f)
                self.get_logger().info(f"🗑️ Deleted local file: {f}")
            except Exception as e:
                self.get_logger().warn(f"Could not delete {f}: {e}")

        # Incrémente l'index et réinitialise
        self.bag_index += 1
        self.initialize_writer()

def main(args=None):
    rclpy.init(args=args)
    node = LiDARRecorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

