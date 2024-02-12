# distortion: plumb_bob
# d:
# - 0.0
# - 0.0
# - 0.0
# - 0.0
# - 0.0
# k:
# - 1696.802685832259
# - 0.0
# - 960.5
# - 0.0
# - 1696.802685832259
# - 540.5
# - 0.0
# - 0.0
# - 1.0


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import time 
class SetModelStatePublisher(Node):
    def __init__(self):
        super().__init__('set_model_state_publisher')
        self.publisher = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publish_model_state)
        self.model_state = ModelState()

    def publish_model_state(self):
        x_values = [0.0, 0.09987, -0.1111]
        y_values = [0.0, 0.1421, -0.1404]
        z_values = [0.1, 0.3, 0.35]
        skew_values = [0.0, -0.2, 0.2]

        for x in x_values:
            for y in y_values:
                for z in z_values:
                    for skew in skew_values:
                        self.model_state.model_name = 'checkerboard'
                        self.model_state.pose = Pose()
                        self.model_state.pose.position.x = x
                        self.model_state.pose.position.y = y
                        self.model_state.pose.position.z = z
                        self.model_state.pose.orientation.x = skew
                        self.model_state.pose.orientation.y = skew
                        self.model_state.pose.orientation.z = skew
                        self.model_state.pose.orientation.w = skew
                        self.model_state.twist = Twist()
                        self.model_state.reference_frame = 'world'

                        # Publish the model state
                        self.publisher.publish(self.model_state)
                        self.get_logger().info(f"Publishing model state:\n{self.model_state}")

                        # Sleep for 0.8 seconds
                        time.sleep(0.8)


def main(args=None):
    rclpy.init(args=args)

    set_model_state_publisher = SetModelStatePublisher()

    rclpy.spin(set_model_state_publisher)

    set_model_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
