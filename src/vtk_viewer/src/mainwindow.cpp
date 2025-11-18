#include "mainwindow.h"
#include <QVBoxLayout>
#include <cmath>

// --- NEW INCLUDES FOR PICKING ---
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkCellPicker.h>
#include <vtkSphereSource.h>

// =========================================================
// Custom Interactor Style Class
// Handles Shift + Click for picking points
// =========================================================
class ClickInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static ClickInteractorStyle* New();
  vtkTypeMacro(ClickInteractorStyle, vtkInteractorStyleTrackballCamera)

  // Pointer to the main window so we can call addSelectedPoint
  MainWindow* main_window_ptr = nullptr;

  virtual void OnLeftButtonDown() override
  {
    // Check if Shift key is held down
    if (this->Interactor->GetShiftKey()) 
    {
      // Get mouse position
      int* clickPos = this->Interactor->GetEventPosition();

      // Create a picker to find what we clicked on
      vtkNew<vtkCellPicker> picker;
      picker->SetTolerance(0.0005);

      // Ray-cast from mouse position into the scene
      picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());

      // Get the exact 3D world coordinates
      double* worldPos = picker->GetPickPosition();
      
      // Get the actor we clicked on
      vtkActor* clickedActor = picker->GetActor();

      // Check if we clicked valid geometry AND if it is the Box (not the floor)
      if (clickedActor && main_window_ptr && main_window_ptr->isTargetBox(clickedActor))
      {
        //Save the point.
        main_window_ptr->addSelectedPoint(worldPos[0], worldPos[1], worldPos[2]);
      }
      else
      {
        printf("Clicked on floor or empty space\n");
      }
    }
    else 
    {
      // No Shift key -> Normal Camera Rotation
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }
  }
};

vtkStandardNewMacro(ClickInteractorStyle);
// =========================================================


MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QMainWindow(parent), ros_node_(node)
{
  is_first_update_ = true;

  this->setWindowTitle("VTK Collision Object Viewer");
  this->setGeometry(100, 100, 800, 600);

  vtk_widget_ = new QVTKOpenGLNativeWidget(this);
  setCentralWidget(vtk_widget_);

  vtk_widget_->renderWindow()->AddRenderer(renderer_);
  renderer_->SetBackground(colors_->GetColor3d("SlateGray").GetData());

  // --- 1. Setup Interactor for Clicking ---
  // Create our custom style
  vtkNew<ClickInteractorStyle> style;
  style->SetDefaultRenderer(renderer_);
  style->main_window_ptr = this; // Give it access to MainWindow functions

  // Attach it to the widget
  vtk_widget_->renderWindow()->GetInteractor()->SetInteractorStyle(style);

  // --- 2. Create Objects ---
  // Target Box
  box_source_->SetXLength(1.0); box_source_->SetYLength(1.0); box_source_->SetZLength(1.0);
  box_source_->SetCenter(0.0, 0.0, 0.0);
  box_mapper_->SetInputConnection(box_source_->GetOutputPort());
  box_actor_->SetMapper(box_mapper_);
  box_actor_->GetProperty()->SetColor(colors_->GetColor3d("MediumSeaGreen").GetData());
  box_actor_->GetProperty()->SetOpacity(0.8);
  renderer_->AddActor(box_actor_);

  // Floor
  floor_source_->SetXLength(1.0); floor_source_->SetYLength(1.0); floor_source_->SetZLength(0.1);
  floor_source_->SetCenter(0.0, 0.0, 0.0);
  floor_mapper_->SetInputConnection(floor_source_->GetOutputPort());
  floor_actor_->SetMapper(floor_mapper_);
  floor_actor_->GetProperty()->SetColor(colors_->GetColor3d("LightGrey").GetData());
  renderer_->AddActor(floor_actor_);

  renderer_->ResetCamera();

  // --- 3. Setup ROS ---
  subscription_ = ros_node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/collision_object", 
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(), 
    std::bind(&MainWindow::topic_callback, this, std::placeholders::_1));

  connect(this, &MainWindow::boxDataReceived, this, &MainWindow::updateVtkBox, Qt::QueuedConnection);
  connect(this, &MainWindow::floorDataReceived, this, &MainWindow::updateVtkFloor, Qt::QueuedConnection);
}

MainWindow::~MainWindow() {}

// --- NEW: Helper to identify the box ---
bool MainWindow::isTargetBox(vtkActor* actor)
{
    return (actor == box_actor_);
}

// --- NEW: Handle the click ---
void MainWindow::addSelectedPoint(double x, double y, double z)
{
    // 1. Save the point to the vector
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    stored_points_.push_back(p);

    RCLCPP_INFO(ros_node_->get_logger(), "Point Saved: [%.3f, %.3f, %.3f] (Total: %zu)", x, y, z, stored_points_.size());

    // 2. Visualize the click (Draw a small Red Sphere)
    vtkNew<vtkSphereSource> sphere;
    sphere->SetCenter(x, y, z);
    sphere->SetRadius(0.015); // Small radius (1.5cm)
    sphere->SetPhiResolution(10); // Lower resolution for performance
    sphere->SetThetaResolution(10);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(sphere->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red color
    
    // Note: In a complex app, we might want to store these actors to delete them later
    renderer_->AddActor(actor);
    
    // Redraw
    vtk_widget_->renderWindow()->Render();
}

void MainWindow::topic_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
{
  if (msg->primitives.empty() || msg->primitives[0].type != shape_msgs::msg::SolidPrimitive::BOX) return;

  const auto& pose = msg->primitive_poses[0];
  const auto& dims = msg->primitives[0].dimensions;
  double roll, pitch, yaw;
  getEulerAngles(pose.orientation, roll, pitch, yaw);

  if (msg->id == "known_target_object") {
    emit boxDataReceived(pose.position.x, pose.position.y, pose.position.z,
                         dims[shape_msgs::msg::SolidPrimitive::BOX_X], dims[shape_msgs::msg::SolidPrimitive::BOX_Y], dims[shape_msgs::msg::SolidPrimitive::BOX_Z],
                         roll, pitch, yaw);
  }
  else if (msg->id == "floor") {
    emit floorDataReceived(pose.position.x, pose.position.y, pose.position.z,
                           dims[shape_msgs::msg::SolidPrimitive::BOX_X], dims[shape_msgs::msg::SolidPrimitive::BOX_Y], dims[shape_msgs::msg::SolidPrimitive::BOX_Z],
                           roll, pitch, yaw);
  }
}

void MainWindow::updateVtkBox(double posX, double posY, double posZ,
                              double dimX, double dimY, double dimZ,
                              double roll, double pitch, double yaw)
{
  box_source_->SetXLength(dimX); box_source_->SetYLength(dimY); box_source_->SetZLength(dimZ);
  box_actor_->SetPosition(posX, posY, posZ);
  box_actor_->SetOrientation(roll, pitch, yaw);

  if (is_first_update_) {
      vtkCamera* camera = renderer_->GetActiveCamera();
      camera->SetFocalPoint(posX, posY, posZ);
      camera->SetPosition(posX - 1.0, posY, posZ + 0.8);
      camera->SetViewUp(0, 0, 1);
      renderer_->ResetCameraClippingRange();
      is_first_update_ = false; 
      RCLCPP_INFO(ros_node_->get_logger(), "Camera auto-focused on object.");
  }
  vtk_widget_->renderWindow()->Render();
}

void MainWindow::updateVtkFloor(double posX, double posY, double posZ,
                                double dimX, double dimY, double dimZ,
                                double roll, double pitch, double yaw)
{
  floor_source_->SetXLength(dimX); floor_source_->SetYLength(dimY); floor_source_->SetZLength(dimZ);
  floor_actor_->SetPosition(posX, posY, posZ);
  floor_actor_->SetOrientation(roll, pitch, yaw);
  vtk_widget_->renderWindow()->Render();
}

void MainWindow::getEulerAngles(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1) pitch = std::copysign(M_PI / 2, sinp);
  else pitch = std::asin(sinp);
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
  roll *= 180.0 / M_PI; pitch *= 180.0 / M_PI; yaw *= 180.0 / M_PI;
}