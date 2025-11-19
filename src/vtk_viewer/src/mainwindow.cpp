#include "mainwindow.h"
#include <cmath>

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkCellPicker.h>
#include <vtkSphereSource.h>
#include <QDir>

// =========================================================
// Custom Interactor Style Class
// Handles Shift + Click for picking points
// =========================================================
class ClickInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static ClickInteractorStyle* New();
  vtkTypeMacro(ClickInteractorStyle, vtkInteractorStyleTrackballCamera)
  MainWindow* main_window_ptr = nullptr;

  virtual void OnLeftButtonDown() override
  {
    if (this->Interactor->GetShiftKey()) {
      int* clickPos = this->Interactor->GetEventPosition();
      vtkNew<vtkCellPicker> picker;
      picker->SetTolerance(0.0005);
      picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());
      double* worldPos = picker->GetPickPosition();
      vtkActor* clickedActor = picker->GetActor();

      if (clickedActor && main_window_ptr && main_window_ptr->isTargetBox(clickedActor)) {
        main_window_ptr->addSelectedPoint(worldPos[0], worldPos[1], worldPos[2]);
      }
    } else {
      vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    }
  }
};
vtkStandardNewMacro(ClickInteractorStyle);


MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent)
  : QMainWindow(parent), ros_node_(node)
{
  is_first_update_ = true;
  this->setWindowTitle("VTK Collision Object Viewer");
  this->setGeometry(100, 100, 800, 600);

  // --- SETUP LAYOUT ---
  QWidget* central_widget = new QWidget(this);
  QVBoxLayout* layout = new QVBoxLayout(central_widget);
  
  vtk_widget_ = new QVTKOpenGLNativeWidget(central_widget);
  layout->addWidget(vtk_widget_);

  // Create a Horizontal Layout for buttons
  QHBoxLayout* button_layout = new QHBoxLayout();
  
  delete_button_ = new QPushButton("Delete Last Point", central_widget);
  button_layout->addWidget(delete_button_);
  
  save_button_ = new QPushButton("Save JSON", central_widget); // <--- NEW BUTTON
  button_layout->addWidget(save_button_);

  layout->addLayout(button_layout);

  setCentralWidget(central_widget);

  // Connect buttons
  connect(delete_button_, &QPushButton::clicked, this, &MainWindow::deleteLastPoint);
  connect(save_button_, &QPushButton::clicked, this, &MainWindow::savePointsToFile); // <--- CONNECT IT

  // --- Setup VTK Pipeline ---
  vtk_widget_->renderWindow()->AddRenderer(renderer_);
  renderer_->SetBackground(colors_->GetColor3d("SlateGray").GetData());

  // Setup Interactor
  vtkNew<ClickInteractorStyle> style;
  style->SetDefaultRenderer(renderer_);
  style->main_window_ptr = this;
  vtk_widget_->renderWindow()->GetInteractor()->SetInteractorStyle(style);

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

  // --- Setup ROS ---
  subscription_ = ros_node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/collision_object", 
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(), 
    std::bind(&MainWindow::topic_callback, this, std::placeholders::_1));

  connect(this, &MainWindow::boxDataReceived, this, &MainWindow::updateVtkBox, Qt::QueuedConnection);
  connect(this, &MainWindow::floorDataReceived, this, &MainWindow::updateVtkFloor, Qt::QueuedConnection);
}

MainWindow::~MainWindow() {}

bool MainWindow::isTargetBox(vtkActor* actor)
{
    return (actor == box_actor_);
}

// --- addSelectedPoint now stores the actor ---
void MainWindow::addSelectedPoint(double x, double y, double z)
{
    // Save Data
    geometry_msgs::msg::Point p;
    p.x = x; p.y = y; p.z = z;
    stored_points_.push_back(p);

    // Log 
    RCLCPP_INFO(ros_node_->get_logger(), "Point Added. [%.3f, %.3f, %.3f] Total: %zu", x, y, z, stored_points_.size());

    vtkNew<vtkSphereSource> sphere;
    sphere->SetCenter(x, y, z);
    sphere->SetRadius(0.015);
    sphere->SetPhiResolution(10);
    sphere->SetThetaResolution(10);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(sphere->GetOutputPort());

    // Use vtkSmartPointer so we can store it in our vector safely
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red

    renderer_->AddActor(actor);
    
    // Save the actor so we can delete it later!
    point_actors_.push_back(actor);

    vtk_widget_->renderWindow()->Render();
}

// --- deleteLastPoint ---
void MainWindow::deleteLastPoint()
{
    // Check if there is anything to delete
    if (stored_points_.empty()) {
        RCLCPP_WARN(ros_node_->get_logger(), "No points to delete.");
        return;
    }

    // Remove Data
    stored_points_.pop_back();

    // Remove Visual Actor
    if (!point_actors_.empty()) {
        // Get the last actor
        vtkActor* actor_to_remove = point_actors_.back();
        
        // Remove it from the VTK scene
        renderer_->RemoveActor(actor_to_remove);
        
        // Remove it from our storage list
        point_actors_.pop_back();
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Last point removed. Remaining: %zu", stored_points_.size());

    // Redraw the scene
    vtk_widget_->renderWindow()->Render();
}

// --- Save to JSON Function ---
void MainWindow::savePointsToFile()
{
    if (stored_points_.empty()) {
         RCLCPP_WARN(ros_node_->get_logger(), "No points to save!");
         return;
    }
    // Get the current working directory of the node
    // Use the QDir to handle paths safely
    QDir workspaceDir(QDir::currentPath());
    
    // Create the 'data' subdirectory if it doesn't exist
    if (!workspaceDir.exists("data")) {
        workspaceDir.mkdir("data");
    }
    
    // Construct the file path: /ur10/data/selected_points.json
    QString fileName = workspaceDir.filePath("data/selected_points.json");

    // Construct the JSON Array
    QJsonArray pointsArray;
    
    for (const auto& p : stored_points_) {
        QJsonObject pointObject;
        pointObject["x"] = p.x;
        pointObject["y"] = p.y;
        pointObject["z"] = p.z;
        pointsArray.append(pointObject);
    }

    // Write to File
    QJsonDocument saveDoc(pointsArray);
    QFile saveFile(fileName);
    
    if (!saveFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Could not open file for writing at: %s", fileName.toStdString().c_str());
        return;
    }

    saveFile.write(saveDoc.toJson());
    RCLCPP_INFO(ros_node_->get_logger(), "SUCCESS! Saved %zu points to: %s", stored_points_.size(), fileName.toStdString().c_str());
    saveFile.close();
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