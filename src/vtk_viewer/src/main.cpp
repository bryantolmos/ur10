#include "mainwindow.h" // Your main window header
#include <QApplication>   // The Qt application class
#include <thread>         // For creating the ROS 2 spin thread

int main(int argc, char *argv[])
{
  // 1. Initialize ROS 2
  rclcpp::init(argc, argv);

  // 2. Initialize the Qt Application
  QApplication a(argc, argv);
  
  // 3. Create the ROS 2 Node
  // We create it here so we can spin it in a separate thread.
  auto ros_node = std::make_shared<rclcpp::Node>("vtk_viewer_node");

  // 4. Create the MainWindow, passing the node to it
  MainWindow w(ros_node);
  w.show(); // Show the window

  // 5. Create a separate thread to spin the ROS 2 node
  // This is critical so the ROS callbacks (like topic_callback)
  // don't block the main GUI thread.
  std::thread ros_thread([&]() {
    rclcpp::spin(ros_node);
  });
  
  // 6. Run the Qt application's event loop in the main thread
  // This blocks until the GUI window is closed.
  int ret = a.exec();

  // 7. Once the GUI is closed, shut down ROS 2
  rclcpp::shutdown();
  ros_thread.join(); // Wait for the spin thread to finish cleanly
  
  return ret;
}