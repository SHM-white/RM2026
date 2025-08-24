#include "camera_test/camera_viewer.hpp"

CameraViewer::CameraViewer() : Node("camera_viewer"), frame_count_(0), current_fps_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Camera Viewer Node has been started.");
    
    // Initialize timing
    last_time_ = std::chrono::steady_clock::now();
    
    // Create subscription to camera image topic
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        qos,
        std::bind(&CameraViewer::image_callback, this, std::placeholders::_1)
    );
    
    // Create OpenCV window
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    
    RCLCPP_INFO(this->get_logger(), "Waiting for camera images on topic 'image_raw'...");
}

CameraViewer::~CameraViewer()
{
    // Clean up OpenCV window
    cv::destroyWindow(WINDOW_NAME);
    RCLCPP_INFO(this->get_logger(), "Camera Viewer Node has been stopped.");
}

void CameraViewer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        // Convert ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        // Calculate FPS
        calculate_fps();
        
        // Display image with FPS overlay
        display_image_with_fps(cv_ptr->image, current_fps_);
        
        // Wait for key press (1ms timeout)
        int key = cv::waitKey(1) & 0xFF;
        if (key == 27) // ESC key
        {
            RCLCPP_INFO(this->get_logger(), "ESC key pressed. Shutting down...");
            rclcpp::shutdown();
        }
        else if (key == 'q' || key == 'Q')
        {
            RCLCPP_INFO(this->get_logger(), "Q key pressed. Shutting down...");
            rclcpp::shutdown();
        }
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void CameraViewer::calculate_fps()
{
    frame_count_++;
    
    auto current_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time_);
    
    // Update FPS every second
    if (duration.count() >= 1000)
    {
        current_fps_ = static_cast<double>(frame_count_) * 1000.0 / duration.count();
        frame_count_ = 0;
        last_time_ = current_time;
        
        RCLCPP_INFO(this->get_logger(), "Current FPS: %.2f", current_fps_);
    }
}

void CameraViewer::display_image_with_fps(const cv::Mat& image, double fps)
{
    cv::Mat display_image = image.clone();
    
    // Add FPS text overlay
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps + 0.5));
    cv::Point text_position(10, 30);
    cv::Scalar text_color(0, 255, 0); // Green color
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    int thickness = 2;
    
    // Add black background for better text visibility
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(fps_text, font_face, font_scale, thickness, &baseline);
    cv::rectangle(display_image, 
                  cv::Point(text_position.x - 5, text_position.y - text_size.height - 5),
                  cv::Point(text_position.x + text_size.width + 5, text_position.y + baseline + 5),
                  cv::Scalar(0, 0, 0), -1);
    
    // Add FPS text
    cv::putText(display_image, fps_text, text_position, font_face, font_scale, text_color, thickness);
    
    // Add image info
    std::string info_text = "Size: " + std::to_string(image.cols) + "x" + std::to_string(image.rows);
    cv::Point info_position(10, display_image.rows - 10);
    cv::putText(display_image, info_text, info_position, font_face, 0.7, cv::Scalar(255, 255, 255), 1);
    
    // Add instructions
    std::string instruction_text = "Press 'q' or ESC to quit";
    cv::Point instruction_position(10, display_image.rows - 40);
    cv::putText(display_image, instruction_text, instruction_position, font_face, 0.6, cv::Scalar(255, 255, 0), 1);
    
    // Display the image
    cv::imshow(WINDOW_NAME, display_image);
}
