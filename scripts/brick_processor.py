#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_gui_bridge.msg import Brick, BrickArray
from geometry_msgs.msg import Pose, Quaternion
import json
import math
# Configuration Constants
CELL_SIZE = 0.03  # Grid cell size 
Z_HEIGHT = 0.026   # Default placement height 

# World Frame Offsets  
WORLD_X_OFFSET = 0.51
WORLD_Y_OFFSET = -0.12

# Map for GUI string-based orientations to degree values
ORIENTATION_MAP = {
    "default": 0, "horizontal": 90, "rotated": 90, "vertical": 0,
    "inverted": 180, "flipped": 270, "upside_down": 180, "left": 270
}

class BrickProcessor(Node):
    def __init__(self):
        super().__init__('brick_processor')
        # Subscribes to raw JSON strings from the GUI bridge
        self.sub = self.create_subscription(String, '/incoming_bricks', self.listener_callback, 10)
        # Publishes structured BrickArray to the Supervisor 
        self.pub = self.create_publisher(BrickArray, '/processed_bricks', 10)
        self.get_logger().info('--- BRICK PROCESSOR READY ---')

    def calculate_world_coords(self, row, col):
        """Transforms grid indices to world coordinates centered on cell."""
        center_offset = CELL_SIZE / 2.0
        world_x = (WORLD_X_OFFSET + (row * CELL_SIZE) + center_offset)
        world_y = WORLD_Y_OFFSET + (col * CELL_SIZE) + center_offset

        self.get_logger().debug(f'Grid ({row}, {col}) -> World ({world_x:.3f}, {world_y:.3f})')
        return world_x, world_y
    
    def get_quaternion_from_euler(self, yaw_degrees):
        """Converts degrees (yaw) to a Quaternion message."""
        yaw = math.radians(yaw_degrees)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def listener_callback(self, msg):
        try:
            full_data = json.loads(msg.data)
            raw_shapes = full_data.get('shapes', [])
            self.get_logger().info(f'Received {len(raw_shapes)} raw shapes from GUI.')
            if isinstance(raw_shapes, dict):
                raw_shapes = list(raw_shapes.values())

            output_msg = BrickArray()

            for i, shape in enumerate(raw_shapes):
                # Debugging first item to verify incoming GUI schema 
                if i == 0:
                    self.get_logger().debug(f"Shape 0 Keys: {list(shape.keys())}")

                brick = Brick()
        
                # Helper data extraction
                center_data = shape.get('centerCell', {})
                pos_data = shape.get('position', {})
                
                # Deep Search Logic for ID, Type, and Color
                brick.id = str(shape.get('id') or center_data.get('id') or pos_data.get('id') or 'unknown')
                brick.type = str(shape.get('type') or pos_data.get('type') or center_data.get('type') or 'unknown')
                brick.color = str(shape.get('color') or center_data.get('color') or 'unknown')
                brick.layer = int(shape.get('layer') or center_data.get('layer') or 1)

                # Robust Orientation Handling
                raw_or = str(shape.get('orientation', 'default')).lower()
                deg = ORIENTATION_MAP.get(raw_or, 0) if not raw_or.replace('-', '').isdigit() else int(raw_or)
  
                # Grid-to-World Transformation
                r_val = int(center_data.get('row') or pos_data.get('row') or 0)
                c_val = int(center_data.get('col') or pos_data.get('col') or 0)

                world_x, world_y = self.calculate_world_coords(r_val, c_val)
                # Instantiate Point objects for the msg structure 
                # self.get_logger().debug(f'Brick ID: {brick.id}, Type: {brick.type}, Color: {brick.color}, Layer: {brick.layer}, Orientation: {deg}°')
                brick.place_pose = Pose() 
                brick.place_pose.position.x = float(world_x)
                brick.place_pose.position.y = float(world_y)
                brick.place_pose.position.z = float(Z_HEIGHT)
                brick.place_pose.orientation = self.get_quaternion_from_euler(deg)

                brick.pickup_pose = Pose() # Defaulted for perception node to populate 
              
                brick.location = 0 # Initialized for Task Allocation
                output_msg.bricks.append(brick)

            # Sort by layer to ensure structural integrity during assembly 
            output_msg.bricks.sort(key=lambda b: b.layer)
            self.pub.publish(output_msg)
            self.get_logger().info(f'Published {len(output_msg.bricks)} bricks with mapping.')

        except Exception as e:
            self.get_logger().error(f"Processing Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BrickProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()