#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge

class SensorDataListener(Node):
    def __init__(self):

        super().__init__("node_test")
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.PointCloudSub = self.create_subscription(PointCloud2, "/gazebo/pointCloud2", self.PcCallBack, qos_profile)
        self.ImageSub = self.create_subscription(Image, "/camera/image_raw", self.ImageCallBack, qos_profile)
        self.cameraIntrinsic = np.array([[-1696.8802685832259, 0.0, 960.5], [0.0, 1696.802685832259, 540.5], [0.0, 0.0, 1.0]])
        self.cameraDistortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.publisher = self.create_publisher(Image, '/fused_image', 10)
        self.get_logger().info("Created subscriber")
        self.bridge = CvBridge()
        self.pcData = None
        self.imageData = None

    def PcCallBack(self, PC):
        self.pcData = PointCloud2() 
        try:
            if(self.imageData is not None):
                pointList = list(pc2.read_points(PC, field_names=("x", "y", "z"), skip_nans=True))
                points3d = np.array([list(pt) for pt in pointList], dtype=np.float64).reshape(-1, 1, 3)
                points2d = self.project_points(points3d)

                # Draw points with size and color based on distance
                for index, point in enumerate(points2d):
                    x, y = point.ravel()
                    distance = points3d[index, 0, 0]
                    circle_size = self.calculate_circle_size(distance)
                    circle_color = self.calculate_circle_color(distance)
                    if 0 <= x < self.imageData.shape[1] and 0 <= y < self.imageData.shape[0]:
                        cv2.circle(self.imageData, (int(x), int(y)), circle_size, circle_color, -1)

                # Convert the OpenCV image with overlays to a ROS2 image message
                ros_image = self.bridge.cv2_to_imgmsg(self.imageData, encoding="bgr8")
                ros_image.header.frame_id = "camera"
                ros_image.header.stamp = PC.header.stamp
                self.publisher.publish(ros_image)
            else:
                return

        except Exception as e:
            self.get_logger().error(f"Error in PcCallBack: {e}")

    def ImageCallBack(self, ImageData):
        try:
            #self.get_logger().info(f"Received PointCloud: {ImageData.data}")
            self.imageData = self.bridge.imgmsg_to_cv2(ImageData, desired_encoding='bgr8') 
        except Exception as e:
            self.get_logger().error(f"Error in PcCallBack: {e}")
    
    def project_points(self, points_3d):
        rvec = np.array([0, 0, 0], dtype=np.float64)
        tvec = np.array([0, 0, 0], dtype=np.float64)
        transformed_points = np.copy(points_3d)
        transformed_points[:, :, 0] = -points_3d[:, :, 1]  # Inverting Y -> X
        transformed_points[:, :, 1] = points_3d[:, :, 2]   # Z -> Y
        transformed_points[:, :, 2] = -points_3d[:, :, 0]   # X -> Z
        points_2d, _ = cv2.projectPoints(transformed_points, rvec, tvec, self.cameraIntrinsic, self.cameraDistortion)
        return points_2d

    def calculate_circle_size(self, distance):
        # Define a maximum and minimum circle size
        max_circle_size = 20
        min_circle_size = 2
        # Define a maximum distance threshold
        max_distance = 50  # Adjust this value based on your scenario
        # Normalize distance value
        normalized_distance = min(distance / max_distance, 1.0)
        # Calculate circle size (linear scaling)
        circle_size = max_circle_size * (1 - normalized_distance) + min_circle_size
        return int(circle_size)

    def calculate_circle_color(self, distance):
        # Define a maximum distance threshold
        max_distance = 50  # Adjust this value based on your scenario
        # Normalize distance value
        normalized_distance = min(distance / max_distance, 1.0)
        # Calculate color intensity (darker for closer, lighter for farther)
        intensity = int(255 * (1 - normalized_distance))
        color = (intensity, 255, intensity)  # Green color with variable intensity
        return color

def main(args=None):
    rclpy.init(args=args)
    SensorDataInstance = SensorDataListener()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(SensorDataInstance, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
