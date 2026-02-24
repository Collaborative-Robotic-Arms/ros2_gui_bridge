#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# We import the messages FROM THIS SAME PACKAGE
from dual_arms_msgs.msg import BricksArray, Brick
from geometry_msgs.msg import Pose

class MockDetector(Node):
    def __init__(self):
        super().__init__('mock_detector')
        # Publish to the topic expected by assembly_plan
        self.pub = self.create_publisher(BricksArray, '/detected_bricks', 10)
        self.timer = self.create_timer(1.0, self.publish_fake_data)
        self.get_logger().info('--- MOCK DETECTOR (DUAL_ARMS_MSGS) STARTED ---')

    def publish_fake_data(self):
        msg = BricksArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "table_frame"

        # --- FAKE BRICK 1: T-Shape (Type 2) ---
        b1 = Brick()
        b1.id = 1               # Real-world unique ID
        b1.type = Brick.T_BRICK     # Enum 2
        b1.side = Brick.AR4         
        b1.pose.position.x = 0.5
        b1.pose.position.y = 0.2
        b1.pose.position.z = 0.05
        msg.bricks.append(b1)

        # --- FAKE BRICK 2: L-Shape (Type 1) ---
        b2 = Brick()
        b2.id = 2
        b2.type = Brick.L_BRICK     # Enum 1
        b2.side = Brick.ABB         
        b2.pose.position.x = -0.5
        b2.pose.position.y = 0.2
        b2.pose.position.z = 0.05
        msg.bricks.append(b2)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()