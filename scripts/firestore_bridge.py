#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import firebase_admin
from firebase_admin import credentials, firestore
import json
import threading
import os
from ament_index_python.packages import get_package_share_directory

# --- CONFIGURATION ---
# The script will look for this filename in the package 'share' folder
CRED_FILENAME = 'collaborative-robotic-ar-d147a-firebase-adminsdk-fbsvc-b399a1ae6e.json'
COLLECTION_NAME = 'shapes' 

def firestore_serializer(obj):
    if hasattr(obj, 'isoformat'): return obj.isoformat()
    return str(obj)

class FirestoreToROS(Node):
    def __init__(self):
        super().__init__('firestore_bridge_node')
        self.publisher_ = self.create_publisher(String, '/incoming_bricks', 10)
        
        # --- NEW: Flag to ignore the historical database load ---
        self.initial_load_complete = False 
        
        self.get_logger().info('--- FIRESTORE BRIDGE STARTING ---')

        # DYNAMIC PATH: Finds the key in the installed 'share' directory
        try:
            pkg_share = get_package_share_directory('ros2_gui_bridge')
            cred_path = os.path.join(pkg_share, CRED_FILENAME)
        except Exception:
            self.get_logger().error("Could not find package share directory!")
            return

        if not os.path.exists(cred_path):
            self.get_logger().error(f"MISSING KEY FILE at: {cred_path}")
            return

        try:
            if not firebase_admin._apps:
                cred = credentials.Certificate(cred_path)
                firebase_admin.initialize_app(cred)
            self.db = firestore.client()
            self.get_logger().info(f"Connected! Listening to '{COLLECTION_NAME}'...")
        except Exception as e:
            self.get_logger().error(f"Firebase Connection Failed: {e}")
            return

        self.listen_thread = threading.Thread(target=self.start_listening)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def start_listening(self):
        try:
            doc_ref = self.db.collection(COLLECTION_NAME)
            doc_ref.on_snapshot(self.on_update)
        except Exception as e:
            self.get_logger().error(f"Listener Error: {e}")

    def on_update(self, col_snapshot, changes, read_time):
        # --- NEW: Ignore the first snapshot containing all history ---
        if not self.initial_load_complete:
            self.get_logger().info("Skipping historical database entries. Waiting for NEW bricks...")
            self.initial_load_complete = True
            return

        for change in changes:
            if change.type.name in ['ADDED', 'MODIFIED']:
                full_doc = change.document.to_dict()
                msg = String()
                msg.data = json.dumps(full_doc, default=firestore_serializer)
                self.publisher_.publish(msg)
                
                count = len(full_doc.get('shapes', []))
                self.get_logger().info(f"Update received. Forwarding {count} shapes.")

def main(args=None):
    rclpy.init(args=args)
    node = FirestoreToROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()