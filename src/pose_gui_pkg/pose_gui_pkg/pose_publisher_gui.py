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
    def __init__(self, topic_name='/goal_pose'):
        super().__init__('pose_publisher_gui_node')
        
        # Define a QoS profile that matches RViz's Goal Pose
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create the publisher with configurable topic
        self.publisher_ = self.create_publisher(
            PoseStamped,
            topic_name,
            qos_profile
        )
        self.get_logger().info(f'Pose Publisher Node started on topic: {topic_name}')

    def publish_pose(self, x, y, z, ox, oy, oz, ow):
        try:
            pose_msg = PoseStamped()
            
            # Fill the header
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'  # Or 'base_link', 'map', etc.
            
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
            self.get_logger().info(f'Publishing Goal Pose: Position[{x}, {y}, {z}] Orientation[w:{ow}, x:{ox}, y:{oy}, z:{oz}]')
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
        self.geometry("450x450")
        
        # Set up the theme
        self.style = ttk.Style(self)
        self.style.theme_use('clam')
        
        # Configure grid
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=2)
        
        self.entries = {}
        
        # --- Position ---
        pos_frame = ttk.LabelFrame(self, text="Position (m)", padding=(10, 5))
        pos_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        pos_frame.columnconfigure(1, weight=1)
        
        self.create_entry(pos_frame, "X:", "0.5", 0, "pos_x")
        self.create_entry(pos_frame, "Y:", "0.0", 1, "pos_y")
        self.create_entry(pos_frame, "Z:", "0.5", 2, "pos_z")
        
        # --- Orientation (Quaternion) ---
        ori_frame = ttk.LabelFrame(self, text="Orientation (Quaternion)", padding=(10, 5))
        ori_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=5, sticky="ew")
        ori_frame.columnconfigure(1, weight=1)

        self.create_entry(ori_frame, "W:", "1.0", 0, "ori_w")
        self.create_entry(ori_frame, "X:", "0.0", 1, "ori_x")
        self.create_entry(ori_frame, "Y:", "0.0", 2, "ori_y")
        self.create_entry(ori_frame, "Z:", "0.0", 3, "ori_z")
        
        # --- Info Label ---
        info_label = ttk.Label(
            self, 
            text="Note: Quaternion must be normalized (w²+x²+y²+z²=1)",
            font=('TkDefaultFont', 8),
            foreground='gray'
        )
        info_label.grid(row=2, column=0, columnspan=2, padx=10, pady=(0, 5), sticky="w")
        
        # --- Publish Button ---
        self.publish_button = ttk.Button(
            self,
            text="Publish the Goal Pose",
            command=self.on_publish  # FIXED: Single underscore
        )
        self.publish_button.grid(row=3, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        
        # --- Status Label ---
        self.status_label = ttk.Label(self, text="Status: Ready", anchor="w")
        self.status_label.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

    def create_entry(self, parent, label_text, default_value, row, key):
        label = ttk.Label(parent, text=label_text, width=8)
        label.grid(row=row, column=0, padx=5, pady=5, sticky="e")
        
        entry = ttk.Entry(parent, width=20)
        entry.insert(0, default_value)
        entry.grid(row=row, column=1, padx=5, pady=5, sticky="ew")
        
        self.entries[key] = entry
        return entry

    def on_publish(self):
        try:
            # Get values using unique keys
            x = self.entries['pos_x'].get()
            y = self.entries['pos_y'].get()
            z = self.entries['pos_z'].get()
            
            ow = self.entries['ori_w'].get()
            ox = self.entries['ori_x'].get()
            oy = self.entries['ori_y'].get()
            oz = self.entries['ori_z'].get()
            
            # Validate quaternion normalization
            try:
                qw, qx, qy, qz = float(ow), float(ox), float(oy), float(oz)
                norm = (qw**2 + qx**2 + qy**2 + qz**2) ** 0.5
                if abs(norm - 1.0) > 0.01:  # Allow small tolerance
                    self.status_label.config(
                        text=f"Warning: Quaternion not normalized (norm={norm:.3f})", 
                        foreground="orange"
                    )
            except ValueError:
                pass  # Will be caught by publish_pose
            
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
    
    topic_name = '/goal_pose'
    
    # Create the ROS node
    ros_node = PosePublisherNode(topic_name=topic_name)
    
    # Set up and run the GUI
    app = PosePublisherGUI(ros_node)
    
    # Run the ROS node spinning in a separate thread
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