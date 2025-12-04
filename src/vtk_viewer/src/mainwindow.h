#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vector>

// --- Qt Includes ---
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
// --- Saving to JSON ---
#include <QFileDialog> // For saving
#include <QJsonDocument> // For JSON
#include <QJsonArray>
#include <QJsonObject>
#include <QFile>

#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "geometry_msgs/msg/point.hpp"

// new includes
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// VTK readers for meshes
#include <vtkSTLReader.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>

#include <QTimer>
#include <mutex>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
  ~MainWindow();

  void addSelectedPoint(double x, double y, double z);
  bool isTargetBox(vtkActor* actor);

  using LinkPoseMap = std::map<std::string, Eigen::Isometry3d>;

private:
  // --- Qt GUI Elements ---
  QVTKOpenGLNativeWidget* vtk_widget_;
  QPushButton* delete_button_;
  QPushButton* save_button_; 

  // --- VTK Members ---
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkNamedColors> colors_;

  // Target Box
  vtkNew<vtkActor> box_actor_;
  vtkNew<vtkCubeSource> box_source_;
  vtkNew<vtkPolyDataMapper> box_mapper_;

  // Floor
  vtkNew<vtkActor> floor_actor_;
  vtkNew<vtkCubeSource> floor_source_;
  vtkNew<vtkPolyDataMapper> floor_mapper_;

  // --- Point Storage ---
  std::vector<geometry_msgs::msg::Point> stored_points_;
  std::vector<vtkSmartPointer<vtkActor>> point_actors_;

  // --- ROS 2 Members ---
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr subscription_;
  
  bool is_first_update_;

  void topic_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);
  void getEulerAngles(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);

  // ROBOT VISUALIZATION MEMBERS
  robot_model_loader::RobotModelLoaderPtr model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;

  // maps link name -> VTK actor
  std::map<std::string, vtkSmartPointer<vtkActor>> robot_link_actors_;

  // subscription for joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  // helper functions
  void setupRobot();
  std::string resolvePackagePath(const std::string& path);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  vtkSmartPointer<vtkPolyData> meshToVtk(const shapes::Mesh* mesh);

  std::mutex data_mutex_;
  LinkPoseMap latest_link_poses_;
  bool new_data_available_ = false; 

  QTimer* render_timer_;

  void drawTimerCallback();
  
signals:
  void boxDataReceived(double posX, double posY, double posZ,
                       double dimX, double dimY, double dimZ,
                       double roll, double pitch, double yaw);
  void floorDataReceived(double posX, double posY, double posZ,
                         double dimX, double dimY, double dimZ,
                         double roll, double pitch, double yaw);
private slots:
  void updateVtkBox(double posX, double posY, double posZ,
                    double dimX, double dimY, double dimZ,
                    double roll, double pitch, double yaw);
  void updateVtkFloor(double posX, double posY, double posZ,
                      double dimX, double dimY, double dimZ,
                      double roll, double pitch, double yaw);
                      
  void deleteLastPoint();
  void savePointsToFile(); 
};
#endif