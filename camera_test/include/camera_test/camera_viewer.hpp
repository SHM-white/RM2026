#ifndef CAMERA_TEST__CAMERA_VIEWER_HPP_
#define CAMERA_TEST__CAMERA_VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

class CameraViewer : public rclcpp::Node
{
public:
    CameraViewer();
    ~CameraViewer();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void calculate_fps();
    void display_image_with_fps(const cv::Mat& image, double fps);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    
    // FPS calculation variables
    std::chrono::steady_clock::time_point last_time_;
    int frame_count_;
    double current_fps_;
    
    // OpenCV window name
    static constexpr const char* WINDOW_NAME = "Camera Test - Image Display";
};

#endif  // CAMERA_TEST__CAMERA_VIEWER_HPP_
