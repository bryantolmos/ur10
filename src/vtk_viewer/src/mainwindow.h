#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVTKOpenGLNativeWidget.h> // The Qt widget for VTK
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkNew.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>

#include "rclcpp/rclcpp.hpp"
#include "moveit_msgs/msg/collision_object.hpp"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(rclcpp::Node::SharedPtr node, QWidget *parent = nullptr);
  ~MainWindow();

private:
  // --- Qt and VTK Members ---
  QVTKOpenGLNativeWidget* vtk_widget_;
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkActor> box_actor_;
  vtkNew<vtkCubeSource> box_source_;
  vtkNew<vtkPolyDataMapper> box_mapper_;
  vtkNew<vtkNamedColors> colors_;

  // --- ROS 2 Members ---
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr subscription_;
  

  void topic_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg);


  void getEulerAngles(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);

signals:

  void boxDataReceived(double posX, double posY, double posZ,
                       double dimX, double dimY, double dimZ,
                       double roll, double pitch, double yaw);

private slots:

  void updateVtkBox(double posX, double posY, double posZ,
                    double dimX, double dimY, double dimZ,
                    double roll, double pitch, double yaw);
};
#endif 