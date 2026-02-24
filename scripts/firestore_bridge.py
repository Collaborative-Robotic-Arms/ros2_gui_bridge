#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import firebase_admin
from firebase_admin import credentials, firestore
import json
import os
from ament_index_python.packages import get_package_share_directory

# --- CONFIGURATION ---
# The script looks for this filename in the package 'share' folder
CRED_FILENAME = 'collaborative-robotic-ar-d147a-bf161ad829c0.json'
COLLECTION_NAME = 'shapes' 

def firestore_serializer(obj):
    if hasattr(obj, 'isoformat'): return obj.isoformat()
    return str(obj)

class FirestoreToROS(Node):
    def __init__(self):
        super().__init__('firestore_bridge_node')
        self.publisher_ = self.create_publisher(String, '/incoming_bricks', 10)
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
            # Initialize Firebase
            if not firebase_admin._apps:
                cred = credentials.Certificate(cred_path)
                firebase_admin.initialize_app(cred)
            self.db = firestore.client()
            
            # Start the listener directly
            self.start_listening()
            self.get_logger().info(f"Connected to Cloud! Monitoring collection: '{COLLECTION_NAME}'")
        except Exception as e:
            self.get_logger().error(f"Firebase Connection Failed: {e}")

    def start_listening(self):
        try:
            doc_ref = self.db.collection(COLLECTION_NAME)
            # IMPORTANT: Save this to a class variable to prevent it from being deleted
            self.snapshot_watch = doc_ref.on_snapshot(self.on_update)
            self.get_logger().info("Real-time snapshot listener is now active.")
        except Exception as e:
            self.get_logger().error(f"Listener Error: {e}")

    def on_update(self, col_snapshot, changes, read_time):
        """Callback triggered whenever the Firestore collection changes."""
        # Heartbeat log to prove the cloud is talking to the node
        self.get_logger().info(f"--- EVENT DETECTED: {len(changes)} documents updated ---")

        for change in changes:
            if change.type.name in ['ADDED', 'MODIFIED']:
                doc_id = change.document.id
                full_doc = change.document.to_dict()
                
                # Publish the full JSON to the topic
                msg = String()
                msg.data = json.dumps(full_doc, default=firestore_serializer)
                self.publisher_.publish(msg)
                
                # Verify if 'shapes' exists as a field inside the document
                shapes_data = full_doc.get('shapes', [])
                count = len(shapes_data) if isinstance(shapes_data, list) else 0
                
                self.get_logger().info(f"Doc [{doc_id}]: Forwarded {count} shapes to /incoming_bricks")

def main(args=None):
    rclpy.init(args=args)
    node = FirestoreToROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()