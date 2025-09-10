import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, PointCloud2
from geometry_msgs.msg import WrenchStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer


class Data_collector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # === ZED External Camera ===
        # 1. Subscriptions for synchronized topics (PointCloud2 + Image)
        self.ext_pointcloud_sub = Subscriber(self, PointCloud2, '/cam_ext/zed/point_cloud/cloud_registered')
        self.ext_image_sub = Subscriber(self, Image, '/cam_ext/zed/left/image_rect_color')

        # 2. Synchronizer for image + point cloud
        self.sync_ext = ApproximateTimeSynchronizer(
            [self.ext_pointcloud_sub, self.ext_image_sub],
            queue_size=30,
            slop=0.05
        )
        self.sync_ext.registerCallback(self.ext_sensor_callback)

        # 3. Separate subscription for IMU (not synchronized)
        self.ext_imu_sub = Subscriber(self, Imu, '/cam_ext/zed/imu/data')
        self.ext_imu_sub.registerCallback(self.ext_imu_callback)

        # === ZED Internal Camera ===
        self.int_pointcloud_sub = Subscriber(self, PointCloud2, '/cam_int/zed/point_cloud/cloud_registered')
        self.int_imu_sub = Subscriber(self, Imu, '/cam_int/zed/imu/data')
        self.int_image_sub = Subscriber(self, Image, '/cam_int/zed/left/image_rect_color')

        self.sync_int = ApproximateTimeSynchronizer(
            [self.int_pointcloud_sub, self.int_imu_sub, self.int_image_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync_int.registerCallback(self.int_sensor_callback)

        # === SenseOne Force Sensor (Separate IMU and Wrench) ===
        self.sense_imu_sub = Subscriber(self, Imu, '/Senseone_eth/imu')
        self.sense_wrench_sub = Subscriber(self, WrenchStamped, '/Senseone_eth/wrench')

        self.sense_imu_sub.registerCallback(self.sense_imu_callback)
        self.sense_wrench_sub.registerCallback(self.sense_wrench_callback)

        self.get_logger().info("Data collector node initialized.")

    # External camera synchronized callback (PointCloud2 + Image)
    def ext_sensor_callback(self, pointcloud_msg, image_msg):
        timestamp = pointcloud_msg.header.stamp
        self.get_logger().info(f"Timestamp: {timestamp.sec}.{timestamp.nanosec}, Collected data: camera_ext")

    # External camera IMU callback (separate)
    def ext_imu_callback(self, imu_msg):
        timestamp = imu_msg.header.stamp
        self.get_logger().info(f"Timestamp: {timestamp.sec}.{timestamp.nanosec}, Collected data: camera_ext_imu")

    # Internal camera synchronized callback (PointCloud2 + IMU + Image)
    def int_sensor_callback(self, pointcloud_msg, imu_msg, image_msg):
        timestamp = pointcloud_msg.header.stamp
        self.get_logger().info(f"Timestamp: {timestamp.sec}.{timestamp.nanosec}, Collected data: camera_int")

    # SenseOne IMU callback
    def sense_imu_callback(self, imu_msg):
        timestamp = imu_msg.header.stamp
        self.get_logger().info(f"Timestamp: {timestamp.sec}.{timestamp.nanosec}, Collected data: senseone_imu")

    # SenseOne wrench callback
    def sense_wrench_callback(self, wrench_msg):
        timestamp = wrench_msg.header.stamp
        self.get_logger().info(f"Timestamp: {timestamp.sec}.{timestamp.nanosec}, Collected data: senseone_wrench")


def main(args=None):
    rclpy.init(args=args)
    node = Data_collector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
