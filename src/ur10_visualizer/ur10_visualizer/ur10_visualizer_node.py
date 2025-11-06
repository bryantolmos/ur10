#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame)
from PyQt5.QtCore import QTimer
from rviz_common.ros_integration import RosNodeAbstraction
from rviz_common.visualization_frame import VisualizationFrame


class RobotVisualizerGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("UR10 Robot Visualizer - Point Selection")
        self.setGeometry(100, 100, 1400, 900)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create top control panel
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)
        
        # Create RViz widget
        self.rviz_frame = VisualizationFrame()
        
        # Set up RViz node abstraction for Jazzy
        ros_node_abstraction = RosNodeAbstraction(self.node)
        self.rviz_frame.setApp(ros_node_abstraction)
        
        # Initialize RViz
        self.rviz_frame.initialize()
        
        # Add RViz frame to layout
        main_layout.addWidget(self.rviz_frame)
        
        # Set up RViz configuration for robot visualization
        self.setup_rviz_config()
        
        # Create timer for ROS spinning
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)  # 100 Hz
        
        # Status label
        self.status_label = QLabel("Status: Ready - Visualizing UR10 Robot Arm")
        self.status_label.setStyleSheet("background-color: #e8f5e9; padding: 8px; border-radius: 4px;")
        main_layout.addWidget(self.status_label)
        
        self.node.get_logger().info("Robot Visualizer GUI started")
    
    def create_control_panel(self):
        """Create the top control panel with buttons"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.StyledPanel)
        panel.setMaximumHeight(80)
        panel.setStyleSheet("background-color: #f5f5f5;")
        
        layout = QHBoxLayout(panel)
        
        # Title
        title = QLabel("🤖 UR10 Robot Arm Visualizer")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #1976d2;")
        layout.addWidget(title)
        
        layout.addStretch()
        
        # Show/Hide Robot button
        self.robot_toggle_btn = QPushButton("Hide Robot")
        self.robot_toggle_btn.clicked.connect(self.toggle_robot_visibility)
        self.robot_toggle_btn.setStyleSheet("padding: 8px 16px;")
        layout.addWidget(self.robot_toggle_btn)
        
        # Show/Hide TF button
        self.tf_toggle_btn = QPushButton("Hide TF")
        self.tf_toggle_btn.clicked.connect(self.toggle_tf_visibility)
        self.tf_toggle_btn.setStyleSheet("padding: 8px 16px;")
        layout.addWidget(self.tf_toggle_btn)
        
        # Reset view button
        reset_btn = QPushButton("Reset View")
        reset_btn.clicked.connect(self.reset_view)
        reset_btn.setStyleSheet("padding: 8px 16px; background-color: #4CAF50; color: white;")
        layout.addWidget(reset_btn)
        
        return panel
    
    def setup_rviz_config(self):
        """Set up the RViz configuration for UR10 robot visualization"""
        # Get the display manager
        manager = self.rviz_frame.getManager()
        
        # Set fixed frame
        manager.setFixedFrame("world")
        
        # Add Grid display
        self.grid_display = manager.createDisplay("rviz_default_plugins/Grid", "Grid", True)
        if self.grid_display:
            self.grid_display.subProp("Plane Cell Count").setValue(20)
            self.grid_display.subProp("Cell Size").setValue(0.5)
            self.grid_display.subProp("Color").setValue("160; 160; 160")
            self.node.get_logger().info("Grid display created")
        
        # Add RobotModel display for UR10
        self.robot_display = manager.createDisplay("rviz_default_plugins/RobotModel", "UR10_Robot", True)
        if self.robot_display:
            self.robot_display.subProp("Description Topic").setValue("/robot_description")
            self.robot_display.subProp("Visual Enabled").setValue(True)
            self.robot_display.subProp("Collision Enabled").setValue(False)
            self.robot_display.subProp("Alpha").setValue(1.0)
            self.node.get_logger().info("RobotModel display created")
        
        # Add TF display
        self.tf_display = manager.createDisplay("rviz_default_plugins/TF", "TF", True)
        if self.tf_display:
            self.tf_display.subProp("Show Names").setValue(True)
            self.tf_display.subProp("Show Axes").setValue(True)
            self.tf_display.subProp("Show Arrows").setValue(False)
            self.tf_display.subProp("Frame Timeout").setValue(15.0)
            self.tf_display.subProp("Scale").setValue(0.3)
            self.node.get_logger().info("TF display created")
        
        # Set initial camera view (Orbit controller)
        view_manager = self.rviz_frame.getManager().getViewManager()
        view_manager.setCurrentViewControllerType("rviz_default_plugins/Orbit")
        
        # Get the view controller and set initial position
        view_controller = view_manager.getCurrent()
        if view_controller:
            # Set distance and viewing angle
            view_controller.subProp("Distance").setValue(3.0)
            view_controller.subProp("Yaw").setValue(0.785)  # 45 degrees
            view_controller.subProp("Pitch").setValue(0.785)
        
        self.status_label.setText("Status: RViz configured - Visualizing UR10 Robot Arm")
        self.node.get_logger().info("RViz configuration complete")
    
    def toggle_robot_visibility(self):
        """Toggle robot visibility"""
        if self.robot_display:
            is_enabled = self.robot_display.isEnabled()
            self.robot_display.setEnabled(not is_enabled)
            self.robot_toggle_btn.setText("Show Robot" if is_enabled else "Hide Robot")
            self.status_label.setText(f"Status: Robot {'hidden' if is_enabled else 'visible'}")
    
    def toggle_tf_visibility(self):
        """Toggle TF visibility"""
        if self.tf_display:
            is_enabled = self.tf_display.isEnabled()
            self.tf_display.setEnabled(not is_enabled)
            self.tf_toggle_btn.setText("Show TF" if is_enabled else "Hide TF")
            self.status_label.setText(f"Status: TF frames {'hidden' if is_enabled else 'visible'}")
    
    def reset_view(self):
        """Reset the RViz camera view"""
        view_manager = self.rviz_frame.getManager().getViewManager()
        view_controller = view_manager.getCurrent()
        if view_controller:
            view_controller.reset()
            view_controller.subProp("Distance").setValue(3.0)
            view_controller.subProp("Yaw").setValue(0.785)
            view_controller.subProp("Pitch").setValue(0.785)
            self.status_label.setText("Status: View reset")
    
    def spin_ros(self):
        """Spin ROS node"""
        rclpy.spin_once(self.node, timeout_sec=0)
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.timer.stop()
        self.node.get_logger().info("Shutting down Robot Visualizer GUI")
        event.accept()


class RobotVisualizerNode(Node):
    def __init__(self):
        super().__init__('robot_visualizer_node')
        self.get_logger().info("Robot Visualizer Node started")


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create ROS node
    node = RobotVisualizerNode()
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create and show GUI
    gui = RobotVisualizerGUI(node)
    gui.show()
    
    # Run application
    exit_code = app.exec_()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()