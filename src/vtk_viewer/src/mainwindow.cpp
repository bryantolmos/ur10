#include "mainwindow.h"
#include <QVBoxLayout>
#include <cmath> // For M_PI and math functions

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QMainWindow(parent), ros_node_(node)
{
  // --- 1. Set up the Qt Window and VTK Widget ---
  this->setWindowTitle("VTK Collision Object Viewer");
  this->setGeometry(100, 100, 800, 600);

  vtk_widget_ = new QVTKOpenGLNativeWidget(this);
  setCentralWidget(vtk_widget_);

  // --- 2. Set up the VTK rendering pipeline ---
  vtk_widget_->GetRenderWindow()->AddRenderer(renderer_);
  renderer_->SetBackground(colors_->GetColor3d("SlateGray").GetData());

  // --- 3. Create the VTK box object ---
  // We initialize it as a simple 1x1x1 cube at (0,0,0)
  // It will be updated when we receive a ROS message
  box_source_->SetXLength(1.0);
  box_source_->SetYLength(1.0);
  box_source_->SetZLength(1.0);
  box_source_->SetCenter(0.0, 0.0, 0.0);

  box_mapper_->SetInputConnection(box_source_->GetOutputPort());
  box_actor_->SetMapper(box_mapper_);
  box_actor_->GetProperty()->SetColor(colors_->GetColor3d("MediumSeaGreen").GetData());
  box_actor_->GetProperty()->SetOpacity(0.8);
  
  renderer_->AddActor(box_actor_);
  renderer_->ResetCamera();

  // --- 4. Set up the ROS 2 Node and Subscriber ---
  // Create the subscriber
  subscription_ = ros_node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/collision_object", 
    // Use a transient_local QoS to get latched messages, just like your Python node
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(), 
    std::bind(&MainWindow::topic_callback, this, std::placeholders::_1));

  // --- 5. Connect the Signal/Slot for thread-safe GUI updates ---
  connect(this, &MainWindow::boxDataReceived, 
          this, &MainWindow::updateVtkBox, 
          Qt::QueuedConnection); // QueuedConnection is ESSENTIAL for cross-thread safety
}

MainWindow::~MainWindow()
{
  // Clean up
}

// Runs in ROS Thread
void MainWindow::topic_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
{
  // *** THIS IS THE CRITICAL FILTER ***
  // We only care about the target object, not the floor.
  if (msg->id != "known_target_object")
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Ignoring CollisionObject with ID: '%s'", msg->id.c_str());
    return;
  }

  // Check if it's a box
  if (msg->primitives.empty() || msg->primitives[0].type != shape_msgs::msg::SolidPrimitive::BOX)
  {
    RCLCPP_WARN(ros_node_->get_logger(), "Received object is not a BOX");
    return;
  }

  // Extract data
  const auto& pose = msg->primitive_poses[0];
  const auto& dims = msg->primitives[0].dimensions;

  // Convert quaternion to Euler angles for VTK
  double roll, pitch, yaw;
  getEulerAngles(pose.orientation, roll, pitch, yaw);

  // Emit the signal to update the GUI
  // This safely sends the data to the main Qt thread
  emit boxDataReceived(
    pose.position.x, pose.position.y, pose.position.z,
    dims[shape_msgs::msg::SolidPrimitive::BOX_X],
    dims[shape_msgs::msg::SolidPrimitive::BOX_Y],
    dims[shape_msgs::msg::SolidPrimitive::BOX_Z],
    roll, pitch, yaw
  );
}

// Runs in Qt GUI Thread
void MainWindow::updateVtkBox(double posX, double posY, double posZ,
                              double dimX, double dimY, double dimZ,
                              double roll, double pitch, double yaw)
{
  RCLCPP_INFO(ros_node_->get_logger(), "Updating VTK Box!");
  
  // 1. Update the box dimensions
  // NOTE: vtkCubeSource defines dimensions, not half-extents.
  box_source_->SetXLength(dimX);
  box_source_->SetYLength(dimY);
  box_source_->SetZLength(dimZ);
  
  // 2. Update the actor's pose
  // vtkCubeSource is centered at (0,0,0), so we move the *actor*
  box_actor_->SetPosition(posX, posY, posZ);
  box_actor_->SetOrientation(roll, pitch, yaw); // VTK uses degrees!

  // 3. Redraw the scene
  renderer_->ResetCamera(); // Auto-focus on the object
  vtk_widget_->GetRenderWindow()->Render();
}

void MainWindow::getEulerAngles(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  // This is a standard quaternion-to-Euler conversion (ZYX order)
  // Roll (x-axis)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // Yaw (z-axis)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);

  // Convert from radians to degrees for VTK
  roll *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  yaw *= 180.0 / M_PI;
}