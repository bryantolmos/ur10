#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h> // Fixed previous error
#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkNew.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vector> 

#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "geometry_msgs/msg/point.hpp" 

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
  ~MainWindow();

  // --- NEW: Public function to handle clicks from the Interactor ---
  void addSelectedPoint(double x, double y, double z);
  
  // --- NEW: Helper to check if an actor is the target box ---
  bool isTargetBox(vtkActor* actor);

private:
  // --- Qt and VTK Members ---
  QVTKOpenGLNativeWidget* vtk_widget_;
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkNamedColors> colors_;

  // -- Target Box Objects --
  vtkNew<vtkActor> box_actor_;
  vtkNew<vtkCubeSource> box_source_;
  vtkNew<vtkPolyDataMapper> box_mapper_;

  // -- Floor Objects --
  vtkNew<vtkActor> floor_actor_;
  vtkNew<vtkCubeSource> floor_source_;
  vtkNew<vtkPolyDataMapper> floor_mapper_;

  // --- NEW: Storage for selected points ---
  std::vector<geometry_msgs::msg::Point> stored_points_;

  // --- ROS 2 Members ---
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr subscription_;
  
  bool is_first_update_;

  void topic_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);
  void getEulerAngles(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);

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
};
#endif