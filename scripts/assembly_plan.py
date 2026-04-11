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

        self.declare_parameter('use_sim', True)
        self.use_sim = self.get_parameter('use_sim').value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ready = True
        self.plan_finalized = False 
        self.table_split_x = 0.453 # Define the X-coordinate that splits the table for side selection       
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
        
        # --- NEW: Track if the plan has already been published ---
        self.plan_published = False 
        
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
        
        if self.use_sim:
            sim_pose = Pose()
            # We use your exact hardware offsets for the simulation table
            sim_pose.position.x = cam_brick.pose.position.y + 0.67
            sim_pose.position.y = cam_brick.pose.position.x + 0.01
            sim_pose.position.z = cam_brick.pose.position.z
            
            # Hardcode orientation to point the gripper down for Gazebo stability
            sim_pose.orientation.x = 0.0
            sim_pose.orientation.y = 1.0
            sim_pose.orientation.z = 0.0
            sim_pose.orientation.w = 0.0
            return sim_pose
        
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'camera'
            pose_stamped.header.stamp = cam_brick.header.stamp

            pose_stamped.pose.position.x = cam_brick.pose.position.y + 0.674
            pose_stamped.pose.position.y = cam_brick.pose.position.x + 0.033
            pose_stamped.pose.orientation = cam_brick.pose.orientation

            return pose_stamped.pose

        except TransformException as ex:
            self.get_logger().error(f'TF2 Failure: {ex}')
            return None


    def gui_callback(self, msg):
        """Stores the list of bricks requested by the GUI."""
        self.required_assembly = msg.bricks
        # --- NEW: Reset the publisher flag because we have a new request ---
        self.plan_published = False 
        self.plan_finalized = False 
        self.latest_valid_plan = []     # Wipe the old plan from memory immediately
        self.update_ready_status(False)
        self.get_logger().info("Received new GUI requirements. Unlocking planner and evaluating...")

    def cam_callback(self, msg):
        """Stores the list of bricks currently detected by the camera."""
        if not self.plan_finalized:
            self.available_supply = msg.bricks
        

    def handle_plan_request(self, request, response):
        if self.is_ready and self.latest_valid_plan:
            final_plan = []
            
            # Same threshold used in validate_and_assign

            for gui_brick in self.latest_valid_plan:
                sup_brick = SupervisorBrick()
                
                sup_brick.id = int(gui_brick.id)
                sup_brick.type = str(gui_brick.type)
                
                sup_brick.pickup_pose = gui_brick.pickup_pose 
                sup_brick.place_pose = gui_brick.place_pose 

                # Where the brick physically is right now
                sup_brick.start_side = "ABB" if gui_brick.location == 0 else "AR4"
                
                # Where the brick needs to end up based on GUI placement
                if sup_brick.place_pose.position.x < self.table_split_x:  # <-- Updated
                    sup_brick.target_side = "ABB"
                else:
                    sup_brick.target_side = "AR4"

                # Log the logic so you can debug handovers easily
                if sup_brick.start_side != sup_brick.target_side:
                    self.get_logger().info(f"HANDOVER REQUIRED for Brick ID {sup_brick.id}: {sup_brick.start_side} -> {sup_brick.target_side}")
                else:
                    self.get_logger().info(f"DIRECT PLACE for Brick ID {sup_brick.id}: {sup_brick.start_side} -> {sup_brick.target_side}")

                final_plan.append(sup_brick)

            response.plan = final_plan
            response.success = True
        else:
            self.get_logger().warn("Supervisor requested plan but system is NOT READY")
            response.success = False
        return response

    def validate_and_assign(self):
        """Matches GUI requirements with Camera reality based on Type and Placement Side."""
        if self.plan_finalized:
            self.update_ready_status(True)
            return
        if not self.tf_ready:
            self.get_logger().warn("Waiting for TF: camera → base_link")
            self.update_ready_status(False)
            return
        
        if not self.required_assembly or not self.available_supply:
            self.get_logger().info("Waiting for GUI requirements or camera supply...", throttle_duration_sec=5.0)
            self.update_ready_status(False)
            return

        supply_pool = list(self.available_supply)
        temp_plan = [] 
        all_matched = True
        

        if not self.plan_published:
            self.get_logger().info(f"Starting validation: {len(self.required_assembly)} bricks required, {len(supply_pool)} available.")

        for gui_target in self.required_assembly:
            target = GuiBrick()
            target.type = gui_target.type
            target.place_pose = gui_target.place_pose

            target_enum_type = self.map_gui_str_to_cam_enum(target.type)
            if target_enum_type is None:
                self.get_logger().error(f"Unknown brick type received from GUI: {target.type}")
                all_matched = False
                continue

            # 1. Determine which side we WANT to pick from based on placement location
            if target.place_pose.position.x < self.table_split_x:  # <-- Updated
                desired_pick_side = CamBrick.ABB
            else:
                desired_pick_side = CamBrick.AR4

            best_source = None
            best_idx = -1

            # PASS 1: Look for an ideal match (Correct Type + Correct Side)
            for idx, source in enumerate(supply_pool):
                if source.type == target_enum_type and source.side == desired_pick_side:
                    best_source = source
                    best_idx = idx
                    break

            # PASS 2: If no ideal match, look for ANY match (This triggers Handover)
            if best_source is None:
                for idx, source in enumerate(supply_pool):
                    if source.type == target_enum_type:
                        best_source = source
                        best_idx = idx
                        break

            # Process the found match
            if best_source is not None:
                target.id = str(best_source.id)   
                abb_pose = self.transform_pose_to_abb(best_source)

                if abb_pose is None:
                    self.get_logger().error(f"TF Transform failed for brick ID: {best_source.id}")
                    all_matched = False
                    break
                
                target.pickup_pose = abb_pose
                # We store the physical start location (0 for ABB, 1 for AR4)
                target.location = 0 if best_source.side == CamBrick.ABB else 1

                temp_plan.append(target)
                supply_pool.pop(best_idx) # Remove from pool so we don't pick it twice
                
                if not self.plan_published:
                    self.get_logger().debug(f"Matched {target.type} (ID: {target.id}) starting at Side: {target.location}")
            else:
                if not self.plan_published:
                    side_str = "ABB" if desired_pick_side == CamBrick.ABB else "AR4"
                    self.get_logger().warn(f"No match found for {target.type} anywhere! (Preferred: {side_str})")
                all_matched = False

        is_ready_now = all_matched and (len(temp_plan) == len(self.required_assembly))
        
        if is_ready_now:
            self.latest_valid_plan = temp_plan
            if not self.plan_published:
                self.get_logger().info(f"Plan validated successfully with {len(temp_plan)} bricks. Publishing plan ONCE.")
                for brick in self.latest_valid_plan:
                    ps = PoseStamped()
                    ps.header.stamp = self.get_clock().now().to_msg()
                    ps.header.frame_id = 'base_link'
                    ps.pose = brick.place_pose
                    self.place_pose_pub.publish(ps)
                
                self.plan_published = True 
                # LOCK THE PLANNER SO IT STOPS LOOKING AT THE CAMERA
                self.plan_finalized = True
        else:
            if self.required_assembly and self.available_supply:
                self.get_logger().warn("Validation failed: Physical supply does not match GUI requirements.", throttle_duration_sec=5.0)
            self.plan_published = False

        self.update_ready_status(is_ready_now)

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