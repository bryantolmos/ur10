#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tkinter as tk
from tkinter import ttk
import threading

# A simple ROS2 node that will publish the pose
class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_gui_node')
        
        # Define a QoS profile that matches RViz's Goal Pose
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create the publisher
        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/goal_pose',  # Standard topic for MoveIt goals
            qos_profile
        )
        self.get_logger().info('Pose Publisher Node has been started.')

    def publish_pose(self, x, y, z, ox, oy, oz, ow):
        try:
            pose_msg = PoseStamped()
            
            # Fill the header
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'  # Or 'base_link', 'map', etc.
            
            # Fill the pose
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = float(z)
            
            pose_msg.pose.orientation.x = float(ox)
            pose_msg.pose.orientation.y = float(oy)
            pose_msg.pose.orientation.z = float(oz)
            pose_msg.pose.orientation.w = float(ow)
            
            # Publish the message
            self.publisher_.publish(pose_msg)
            self.get_logger().info(f'Publishing Goal Pose: [P: {x}, {y}, {z}] [O: {ow}, {ox}, {oy}, {oz}]')
            return True, "Pose published successfully!"
            
        except ValueError as e:
            self.get_logger().error(f'Invalid input: {e}')
            return False, f"Error: Invalid number. {e}"
        except Exception as e:
            self.get_logger().error(f'Failed to publish pose: {e}')
            return False, f"Error: {e}"

# The main Tkinter GUI Application
class PosePublisherGUI(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.title("UR10 Pose Publisher")
        self.geometry("400x400")
        
        # Set up the theme
        self.style = ttk.Style(self)
        self.style.theme_use('clam')
        
        # Configure grid
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=2)
        
        self.entries = {}
        
        # --- Position ---
        pos_frame = ttk.LabelFrame(self, text="Position (meters)", padding=(10, 5))
        pos_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        pos_frame.columnconfigure(1, weight=1)
        
        # *** FIX: Use unique keys for each entry ***
        self.create_entry(pos_frame, "X:", "0.5", 0, "pos_x")
        self.create_entry(pos_frame, "Y:", "0.0", 1, "pos_y")
        self.create_entry(pos_frame, "Z:", "0.5", 2, "pos_z")
        
        # --- Orientation (Quaternion) ---
        ori_frame = ttk.LabelFrame(self, text="Orientation (Quaternion)", padding=(10, 5))
        ori_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="ew")
        ori_frame.columnconfigure(1, weight=1)

        # *** FIX: Use unique keys for each entry ***
        self.create_entry(ori_frame, "W:", "1.0", 0, "ori_w")
        self.create_entry(ori_frame, "X:", "0.0", 1, "ori_x")
        self.create_entry(ori_frame, "Y:", "0.0", 2, "ori_y")
        self.create_entry(ori_frame, "Z:", "0.0", 3, "ori_z")
        
        # --- Publish Button ---
        self.publish_button = ttk.Button(
            self,
            text="Publish Goal Pose",
            command=self.on_publish  # <-- FIX 1: Was self.on__publish
        )
        self.publish_button.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="ew") # <-- FIX 2: Was padx=1a0
        
        # --- Status Label ---
        self.status_label = ttk.Label(self, text="Status: Ready", anchor="w")
        self.status_label.grid(row=3, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

    def create_entry(self, parent, label_text, default_value, row, key): # <-- Added key
        label = ttk.Label(parent, text=label_text)
        label.grid(row=row, column=0, padx=5, pady=5, sticky="e")
        
        entry = ttk.Entry(parent, width=20)
        entry.insert(0, default_value)
        entry.grid(row=row, column=1, padx=5, pady=5, sticky="ew")
        
        self.entries[key] = entry # <-- Use the unique key
        return entry

    def on_publish(self):
        try:
            # *** FIX: Get values using unique keys ***
            x = self.entries['pos_x'].get()
            y = self.entries['pos_y'].get()
            z = self.entries['pos_z'].get()
            
            ow = self.entries['ori_w'].get()
            ox = self.entries['ori_x'].get()
            oy = self.entries['ori_y'].get()
            oz = self.entries['ori_z'].get()
            
            # Call the ROS node's publish method
            success, message = self.ros_node.publish_pose(x, y, z, ox, oy, oz, ow)
            
            if success:
                self.status_label.config(text=f"Status: {message}", foreground="green")
            else:
                self.status_label.config(text=f"Status: {message}", foreground="red")
                
        except Exception as e:
            self.status_label.config(text=f"Status: GUI Error: {e}", foreground="red")

def main(args=None):
    rclpy.init(args=args)
    
    ros_node = PosePublisherNode()
    app = PosePublisherGUI(ros_node)
    
    # Run the ROS node spinning
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    try:
        # Start the Tkinter main loop (must be in the main thread)
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        ros_node.get_logger().info('Shutting down...')
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()