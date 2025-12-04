#include "mainwindow.h"
#include <cmath>

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkCellPicker.h>
#include <vtkSphereSource.h>
#include <QDir>
#include <QTextStream>
#include <geometric_shapes/shapes.h>

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

  qRegisterMetaType<LinkPoseMap>("LinkPoseMap");
  
  // --- SETUP LAYOUT ---
  QWidget* central_widget = new QWidget(this);
  QVBoxLayout* layout = new QVBoxLayout(central_widget);
  
  vtk_widget_ = new QVTKOpenGLNativeWidget(central_widget);
  layout->addWidget(vtk_widget_);

  // Create a Horizontal Layout for buttons
  QHBoxLayout* button_layout = new QHBoxLayout();
  
  delete_button_ = new QPushButton("Delete Last Point", central_widget);
  button_layout->addWidget(delete_button_);
  
  save_button_ = new QPushButton("Save YAML", central_widget); // <--- NEW BUTTON
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

  // Constructor setup, adding the call to setupRobot() in constructor and initializing the joint subscription
  setupRobot();

  joint_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&MainWindow::jointStateCallback, this, std::placeholders::_1));

  render_timer_ = new QTimer(this);
  connect(render_timer_, &QTimer::timeout, this, &MainWindow::drawTimerCallback);
  render_timer_->start(33);
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

// --- Save to YAML Function ---
void MainWindow::savePointsToFile()
{
    if (stored_points_.empty()) {
         RCLCPP_WARN(ros_node_->get_logger(), "No points to save!");
         return;
    }
    // Get the current working directory of the node
    QDir workspaceDir(QDir::currentPath());
    // Create the data subdirectory if it doesn't exist
    if (!workspaceDir.exists("data")) {
        workspaceDir.mkdir("data");
    }
    
    // Construct the file path: /ur10/data/selected_points.yaml
    QString fileName = workspaceDir.filePath("data/selected_points.yaml");

  QFile saveFile(fileName);
    if (!saveFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Could not open file for writing at: %s", fileName.toStdString().c_str());
        return;
    }

    // Write YAML Content
    QTextStream out(&saveFile);
    
    // Header Comment
    out << "# Selected Points from VTK Viewer\n";
    out << "selected_points:\n"; // Root key

    for (const auto& p : stored_points_) {
        // YAML formatting
        out << "  - x: " << p.x << "\n";
        out << "    y: " << p.y << "\n";
        out << "    z: " << p.z << "\n";
    }

    saveFile.close();
    RCLCPP_INFO(ros_node_->get_logger(), "SUCCESS! Saved %zu points to: %s", stored_points_.size(), fileName.toStdString().c_str());
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

// function that iterated over the moveit robot model, finds the mesh files, and loads them into VTK
void MainWindow::setupRobot() {
  // load robot model from ros paremeter "robot_description"
  model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(ros_node_, "robot_description");
  kinematic_model_ = model_loader_->getModel();

  if (!kinematic_model_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "FAILED to load robot mode, is 'robot_description' param set?");
    return;
  }

  kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();

  const auto& links = kinematic_model_->getLinkModels();

  for (const moveit::core::LinkModel* link : links) {
    if (link->getShapes().empty()) continue;

    const std::string& link_name = link->getName();
    auto shape = link->getShapes()[0]; 

    if (shape->type == shapes::MESH) {
      std::string mesh_path = link->getVisualMeshFilename();
      if (mesh_path.empty()) continue;

      std::string fs_path = resolvePackagePath(mesh_path);
      if (fs_path.empty()) continue;

      std::string file_uri = "file://" + fs_path; 
      shapes::Mesh* loaded_mesh = shapes::createMeshFromResource(file_uri);

      if (!loaded_mesh) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to load mesh: %s", fs_path.c_str());
        continue;
      }

      vtkSmartPointer<vtkPolyData> vtk_mesh = meshToVtk(loaded_mesh);
      delete loaded_mesh;

      vtkNew<vtkPolyDataMapper> mapper;
      mapper->SetInputData(vtk_mesh);

      vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      actor->GetProperty()->SetColor(0.7, 0.7, 0.7);
             
      renderer_->AddActor(actor);
      robot_link_actors_[link_name] = actor;
    }
  }
}

std::string MainWindow::resolvePackagePath(const std::string& path) {
    if (path.find("package://") == 0) {
        size_t start = 10;
        size_t end = path.find("/", start);
        std::string pkg_name = path.substr(start, end - start);
        std::string relative_path = path.substr(end);
        
        try {
            std::string pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
            return pkg_path + relative_path;
        } catch (...) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Could not find package: %s", pkg_name.c_str());
            return "";
        }
    }
    return path;
}

void MainWindow::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!kinematic_state_) return;

    kinematic_state_->setVariableValues(*msg);
    kinematic_state_->update();

    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (auto const& [name, actor] : robot_link_actors_) {
        latest_link_poses_[name] = kinematic_state_->getGlobalLinkTransform(name);
    }
    
    new_data_available_ = true;
}

void MainWindow::drawTimerCallback()
{
    LinkPoseMap current_poses;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!new_data_available_) return;
        
        current_poses = latest_link_poses_;
        new_data_available_ = false;
    }

    // update VTK
    for (auto const& [name, pose] : current_poses) {
        if (robot_link_actors_.find(name) == robot_link_actors_.end()) continue;

        vtkNew<vtkMatrix4x4> vtk_mat;
        for(int r=0; r<4; r++) {
            for(int c=0; c<4; c++) {
                vtk_mat->SetElement(r, c, pose(r,c));
            }
        }
        robot_link_actors_[name]->SetUserMatrix(vtk_mat);
    }

    vtk_widget_->renderWindow()->Render();
}

vtkSmartPointer<vtkPolyData> MainWindow::meshToVtk(const shapes::Mesh* mesh)
{
    if (!mesh) return nullptr;

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> triangles;

    for (unsigned int i = 0; i < mesh->vertex_count; ++i)
    {
        points->InsertNextPoint(mesh->vertices[3 * i], 
                                mesh->vertices[3 * i + 1], 
                                mesh->vertices[3 * i + 2]);
    }

    for (unsigned int i = 0; i < mesh->triangle_count; ++i)
    {
        vtkIdType ids[3];
        ids[0] = mesh->triangles[3 * i];
        ids[1] = mesh->triangles[3 * i + 1];
        ids[2] = mesh->triangles[3 * i + 2];
        triangles->InsertNextCell(3, ids);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);
    
    return polyData;
}