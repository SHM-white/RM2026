#include "camera_test/camera_viewer.hpp"

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create and run the camera viewer node
    auto node = std::make_shared<CameraViewer>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
