#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_gui_bridge.msg import Brick, BrickArray
from geometry_msgs.msg import Pose, Quaternion
import json

# =========================
# Configuration Constants
# =========================
CELL_SIZE = 0.03
Z_HEIGHT = 0.026

WORLD_X_OFFSET = 0.51
WORLD_Y_OFFSET = -0.12

ORIENTATION_MAP = {
    "default": -45, "horizontal": 90,
    "rotated": 45, "vertical": 0,
    "inverted": 135, "upside_down": 180,
    "flipped": 270, "left": 270
}

# =========================
# Geometry Helpers
# =========================

def centroid_from_cells(cells):
    rows = [c['row'] for c in cells]
    cols = [c['col'] for c in cells]
    return sum(rows) / len(rows), sum(cols) / len(cols)

def neighbor_count(cell, cell_set):
    r, c = cell
    return sum(
        (r + dr, c + dc) in cell_set
        for dr, dc in [(1,0), (-1,0), (0,1), (0,-1)]
    )

def t_brick_anchor(cells):
    """
    Returns the junction cell (degree-3 node).
    """
    cell_set = {(c['row'], c['col']) for c in cells}

    for cell in cell_set:
        if neighbor_count(cell, cell_set) == 3:
            return cell

    raise ValueError("Invalid T brick: no junction cell found")

def l_brick_bounding_center(cells):
    """
    Center of the bounding rectangle treating the L as a full rectangle.
    """
    rows = [c['row'] for c in cells]
    cols = [c['col'] for c in cells]

    min_r, max_r = min(rows), max(rows)
    min_c, max_c = min(cols), max(cols)

    center_row = (min_r + max_r + 1) / 2
    center_col = (min_c + max_c + 1) / 2

    return center_row, center_col

# =========================
# ROS Node
# =========================

class BrickProcessor(Node):
    def __init__(self):
        super().__init__('brick_processor')

        self.sub = self.create_subscription(
            String, '/incoming_bricks', self.listener_callback, 10
        )
        self.pub = self.create_publisher(
            BrickArray, '/processed_bricks', 10
        )

        self.get_logger().info('--- BRICK PROCESSOR READY ---')

    def grid_to_world(self, row, col , brick_type):
        center_offset = CELL_SIZE / 2.0
        world_x = WORLD_X_OFFSET + (row * CELL_SIZE) + center_offset
        world_y = WORLD_Y_OFFSET + (col * CELL_SIZE) + center_offset

        if brick_type.upper() == 'L_SHAPE':
            world_x = WORLD_X_OFFSET + (row * CELL_SIZE) 
            world_y = WORLD_Y_OFFSET + (col * CELL_SIZE)
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
            # full_data = json.loads(msg.data)
            # raw_shapes = full_data.get('shapes', [])

            # if isinstance(raw_shapes, dict):
            #     raw_shapes = list(raw_shapes.values())
            
            full_data = json.loads(msg.data)
            raw_shapes = full_data.get('shapes', [])
            self.get_logger().info(f'Received {len(raw_shapes)} raw shapes from GUI.')
            if isinstance(raw_shapes, dict):
                raw_shapes = list(raw_shapes.values())

            output_msg = BrickArray()


            for i, shape in enumerate(raw_shapes):

                # 🔍 Raw input debug (first shape only)
                if i == 0:
                    # self.get_logger().debug(
                    #     "Raw shape[0]:\n" + json.dumps(shape, indent=2)
                    # )
                    self.get_logger().info(f"Shape 0 Keys: {list(shape.keys())}")
                    self.get_logger().info(f"Shape 0 type: {shape.get('type', {})}")

                brick = Brick()

                center_data = shape.get('centerCell', {})
                pos_data = shape.get('position', {})

                # -------------------------
                # Metadata extraction
                # -------------------------
                brick.id = str(
                    shape.get('id')
                    or center_data.get('id')
                    or pos_data.get('id')
                    or 'unknown'
                )

                brick.type = str(
                    shape.get('type')
                    or pos_data.get('type')
                    or center_data.get('type')
                    or 'unknown'
                )

                brick.color = str(
                    shape.get('color')
                    or center_data.get('color')
                    or 'unknown'
                )

                brick.layer = int(
                    shape.get('layer')
                    or center_data.get('layer')
                    or 1
                )

                raw_or = str(shape.get('orientation', 'default')).lower()
                self.get_logger().info(f"Shape 0 orientation: {raw_or}")
                if raw_or.replace('-', '').isdigit():
                    brick.orientation = int(raw_or)
                else:
                    # brick.orientation = ORIENTATION_MAP.get(raw_or, 0)
                    deg = ORIENTATION_MAP.get(raw_or, 0) if not raw_or.replace('-', '').isdigit() else int(raw_or)

                # -------------------------
                # Placement logic
                # -------------------------
                if 'occupiedCells' in shape:
                    cells = shape['occupiedCells']
                    self.get_logger().info(f"Cells detected for brick [{brick.id}]: {cells}")
                    brick_type = brick.type.upper()

                    if brick_type == 'I_SHAPE':
                        r_cent, c_cent = centroid_from_cells(cells)
                        source = 'I centroid'

                    elif brick_type == 'T_SHAPE':
                        r_cent, c_cent = t_brick_anchor(cells)
                        source = 'T junction cell'

                    elif brick_type == 'L_SHAPE':
                        r_cent, c_cent = l_brick_bounding_center(cells)
                        source = 'L bounding box center'

                    else:
                        r_cent, c_cent = centroid_from_cells(cells)
                        source = 'default centroid'

                else:
                    r_cent = int(center_data.get('row') or pos_data.get('row') or 0)
                    c_cent = int(center_data.get('col') or pos_data.get('col') or 0)
                    source = 'legacy single cell'

                    self.get_logger().warn(
                        f"[{brick.id}] cells[] missing, using legacy anchor"
                    )

                # 🔍 Debug placement decision
                self.get_logger().info(
                    f"[{brick.id}] placement source: {source}, "
                    f"grid=(row={r_cent}, col={c_cent})"
                )

                world_x, world_y = self.grid_to_world(r_cent, c_cent, brick_type)

                self.get_logger().info(
                    f"[{brick.id}] world coords: x={world_x:.4f}, y={world_y:.4f}"
                )

                # -------------------------
                # Message population
                # -------------------------
              

                brick.place_pose = Pose() 
                brick.place_pose.position.x = float(world_x)
                brick.place_pose.position.y = float(world_y)
                brick.place_pose.position.z = float(Z_HEIGHT)
                brick.place_pose.orientation = self.get_quaternion_from_euler(deg)

                brick.pickup_pose = Pose()
                brick.location = 0

                self.get_logger().debug(
                    f"[{brick.id}] FINAL → type={brick.type}, "
                    f"layer={brick.layer}, orientation={brick.orientation}"
                )

                output_msg.bricks.append(brick)

            output_msg.bricks.sort(key=lambda b: b.layer)
            self.pub.publish(output_msg)

            self.get_logger().info(
                f"Published {len(output_msg.bricks)} bricks"
            )

        except Exception as e:
            self.get_logger().error(f"Processing Error: {str(e)}")

# =========================
# Main
# =========================

def main(args=None):
    rclpy.init(args=args)
    node = BrickProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
