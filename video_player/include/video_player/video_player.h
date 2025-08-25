#ifndef VIDEO_PLAYER_H
#define VIDEO_PLAYER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "camera_interfaces/srv/param_event.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <vector>
#include <string>

enum VideoParams
{
    ALL_PARAMS,
    VIDEO_WIDTH,
    VIDEO_HEIGHT,
    VIDEO_FRAMERATE,
    NONE
};

class VideoPlayerNode : public rclcpp::Node
{
public:
    VideoPlayerNode(const std::string& node_name);
    ~VideoPlayerNode();

    // 主要功能函数
    bool init();
    bool loadVideoFiles();
    bool getNextFrame(cv::Mat& frame);
    void publishImage();
    void run();

    // 参数设置函数
    void getParams();
    void setVideoParams(VideoParams which_param = VideoParams::ALL_PARAMS);
    
    // ROS2 服务和发布器
    void setupPublishers();
    void setupServices();
    void paramServiceCallback(
        const std::shared_ptr<camera_interfaces::srv::ParamEvent::Request> request,
        std::shared_ptr<camera_interfaces::srv::ParamEvent::Response> response);

    // 获取器函数
    double getFramerate() const { return framerate_; }
    
    // 公共成员变量（与相机驱动模块保持一致）
    VideoParams param_change = NONE;

private:
    // ROS2 相关
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::Service<camera_interfaces::srv::ParamEvent>::SharedPtr param_service_;
    
    // 视频相关
    std::vector<std::string> video_files_;
    std::vector<cv::VideoCapture> video_captures_;
    size_t current_video_index_;
    bool loop_videos_;
    
    // 参数
    std::string video_directory_;
    double framerate_;
    int video_width_;
    int video_height_;
    std::string camera_frame_id_;
    
    // 内部状态
    bool initialized_;
    cv::Mat current_frame_;
    
    // 辅助函数
    bool isVideoFile(const std::string& filename) const;
    void resetToFirstVideo();
    sensor_msgs::msg::CameraInfo createCameraInfo();
};

#endif // VIDEO_PLAYER_H
