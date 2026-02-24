#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# --- IMPORT MESSAGES ---
# 1. Source: GUI / Brick Processor Messages
from ros2_gui_bridge.msg import BrickArray as GuiArray
from ros2_gui_bridge.msg import Brick as GuiBrick

# 2. Source: Camera / Detection Messages
from dual_arms_msgs.msg import BricksArray as CamArray
from dual_arms_msgs.msg import Brick as CamBrick 

# 3. Destination: Supervisor Service and Message Types
from supervisor_package.srv import GetAssemblyPlan 
from supervisor_package.msg import SuperBrick as SupervisorBrick 
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs 
from geometry_msgs.msg import Pose,PoseStamped

class AssemblyAllocator(Node):
    def __init__(self):
        super().__init__('assembly_allocator')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ready = True
        # self.tf_check_timer = self.create_timer(0.5, self.check_tf_ready)
        self.place_pose_pub = self.create_publisher(PoseStamped, '/debug/place_pose', 10)


        # --- INPUTS ---
        # Subscribes to the Brick Processor (GUI World Coords)
        self.gui_sub = self.create_subscription(
            GuiArray, '/processed_bricks', self.gui_callback, 10)
        
        # Subscribes to the Detection Mock/Pipeline (Hardware Reality)
        self.cam_sub = self.create_subscription(
            CamArray, '/detected_bricks', self.cam_callback, 10)
        
        # --- OUTPUTS ---
        # Service Server for the Supervisor to fetch the plan
        self.srv = self.create_service(
            GetAssemblyPlan, 
            'get_assembly_plan', 
            self.handle_plan_request
        )
        
        # Debugging publishers
        self.flag_pub = self.create_publisher(Bool, '/debug/assembly_ready', 10)
        
        # Internal State
        self.required_assembly = []   # Bricks from GUI
        self.available_supply = []     # Bricks from Camera
        self.latest_valid_plan = []    # Matched plan ready for supervisor
        self.is_ready = False
        
        # Matching Timer: runs at 2Hz
        self.timer = self.create_timer(0.5, self.validate_and_assign)
        self.get_logger().info('--- ASSEMBLY ALLOCATOR READY ---')

    def check_tf_ready(self):
        if self.tf_buffer.can_transform(
            'base_link',
            'camera_optical_frame',
            rclpy.time.Time()
        ):
            self.tf_ready = True
            self.get_logger().info("TF READY: camera → base_link")
            self.tf_check_timer.cancel()

    def transform_pose_to_abb(self, cam_brick):
        if not self.tf_ready:
            return None
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'camera'
            pose_stamped.header.stamp = cam_brick.header.stamp
            # pose_stamped.pose = cam_brick.pose

            # self.get_logger().info(f"Incoming brick pose: position x={cam_brick.pose.position.x}, y={cam_brick.pose.position.y}")
            # t = self.tf_buffer.lookup_transform(
            #     'base_link',
            #     'camera',
            #     rclpy.time.Time()
            # )
            pose_stamped.pose.position.x = cam_brick.pose.position.y + 0.674
            pose_stamped.pose.position.y = cam_brick.pose.position.x + 0.033

            pose_stamped.pose.orientation = cam_brick.pose.orientation

            # self.get_logger().info(f"Processed brick pose: position x={pose_stamped.pose.position.x}, y={pose_stamped.pose.position.y}")
            
            # transformed = tf2_geometry_msgs.do_transform_pose_stamped(
            #     pose_stamped, t
            # )

            return pose_stamped.pose

        except TransformException as ex:
            self.get_logger().error(f'TF2 Failure: {ex}')
            return None


    def gui_callback(self, msg):
        """Stores the list of bricks requested by the GUI."""
        self.required_assembly = msg.bricks

    def cam_callback(self, msg):
        """Stores the list of bricks currently detected by the camera."""
        self.available_supply = msg.bricks

    def handle_plan_request(self, request, response):
        if self.is_ready and self.latest_valid_plan:
            final_plan = []
            for gui_brick in self.latest_valid_plan:
                sup_brick = SupervisorBrick()
                
                sup_brick.id = int(gui_brick.id)
                sup_brick.type = str(gui_brick.type)
                
                # Use the full Pose objects stored during validate_and_assign
                sup_brick.pickup_pose = gui_brick.pickup_pose 
                sup_brick.place_pose = gui_brick.place_pose 

                sup_brick.start_side = "ABB" if gui_brick.location == 1 else "AR4"
                sup_brick.target_side = "SHARED"
                
                final_plan.append(sup_brick)

            response.plan = final_plan
            response.success = True
        else:
                self.get_logger().warn("Supervisor requested plan but system is NOT READY")
                response.success = False
        return response

    def validate_and_assign(self):
        """Matches GUI requirements with Camera reality based on Type and X-coordinate zones."""
        if not self.tf_ready:
            self.get_logger().warn("Waiting for TF: camera → base_link")
            self.update_ready_status(False)
            return
        if not self.required_assembly or not self.available_supply:
            self.update_ready_status(False)
            return

        supply_pool = list(self.available_supply)
        temp_plan = [] 
        all_matched = True

        for gui_target in self.required_assembly:
            target = GuiBrick()
            target.type = gui_target.type
            target.place_pose = gui_target.place_pose

            match_found = False
            target_enum_type = self.map_gui_str_to_cam_enum(target.type)
            
            if target_enum_type is None:
                all_matched = False
                continue

            # --- NEW LOGIC: Determine required side based on X coordinate ---
            required_side = None
            if target.place_pose.position.x <= 0.585:
                required_side = CamBrick.ABB
            elif target.place_pose.position.x >= 0.675:
                required_side = CamBrick.AR4
            else:
                # This is the "Dead Zone" between 0.585 and 0.675
                self.get_logger().error(
                    f"UNABLE TO REACH BRICK: Target X {target.place_pose.position.x} is in the dead zone!"
                )
                all_matched = False
                continue 
            # ----------------------------------------------------------------

            for idx, source in enumerate(supply_pool):
                # Match Type AND Side (if side requirement exists)
                type_matches = (source.type == target_enum_type)
                side_matches = (required_side is None or source.side == required_side)

                if type_matches and side_matches:
                    target.id = str(source.id)   

                    abb_pose = self.transform_pose_to_abb(source)

                    # self.get_logger().info(f"abb pose: position x={abb_pose.position.x}, y={abb_pose.position.y}")
                    if abb_pose is None:
                        all_matched = False
                        break
                    
                    target.pickup_pose = abb_pose

                    # Map side back to location integer for the internal plan
                    if source.side == CamBrick.ABB:
                        target.location = 1
                    elif source.side == CamBrick.AR4:
                        target.location = 2

                    temp_plan.append(target)
                    supply_pool.pop(idx)
                    match_found = True
                    break

            if not match_found:
                # If no brick matches both type and the X-zone requirement
                all_matched = False

        # System is only ready if EVERY brick from the GUI has a physical match
        is_ready_now = all_matched and (len(temp_plan) == len(self.required_assembly))
        self.update_ready_status(is_ready_now)
        if self.is_ready:
            self.latest_valid_plan = temp_plan
            for brick in self.latest_valid_plan:
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = 'base_link'  # Make sure this matches your robot/world frame
                ps.pose = brick.place_pose
                self.place_pose_pub.publish(ps)

    def update_ready_status(self, status):
        """Publishes the ready flag for debugging."""
        self.is_ready = status
        flag_msg = Bool()
        flag_msg.data = self.is_ready
        self.flag_pub.publish(flag_msg)

    def map_gui_str_to_cam_enum(self, gui_string):
        """Maps shape strings to camera message enum values."""
        mapping = {
            "i_shape": CamBrick.I_BRICK, "I_shape": CamBrick.I_BRICK,
            "l_shape": CamBrick.L_BRICK, "L_shape": CamBrick.L_BRICK,
            "t_shape": CamBrick.T_BRICK, "T_shape": CamBrick.T_BRICK,
            "z_shape": CamBrick.Z_BRICK, "Z_shape": CamBrick.Z_BRICK
        }
        val = mapping.get(gui_string)
        if val is None:
            self.get_logger().error(f"Unknown GUI brick type: {gui_string}")
        return val

def main(args=None):
    rclpy.init(args=args)
    node = AssemblyAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()