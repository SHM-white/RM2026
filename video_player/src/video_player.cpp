#include "video_player/video_player.h"
#include <filesystem>
#include <algorithm>

VideoPlayerNode::VideoPlayerNode(const std::string& node_name) 
    : Node(node_name), 
      current_video_index_(0),
      loop_videos_(true),
      framerate_(30.0),
      video_width_(640),
      video_height_(480),
      camera_frame_id_("camera"),
      initialized_(false)
{
    RCLCPP_INFO(this->get_logger(), "创建视频播放节点: %s", node_name.c_str());
    
    // 获取参数
    getParams();
    
    // 设置发布器和服务
    setupPublishers();
    setupServices();
    
    // 初始化
    if (!init()) {
        RCLCPP_ERROR(this->get_logger(), "视频播放器初始化失败！");
    }
}

VideoPlayerNode::~VideoPlayerNode()
{
    // 清理资源
    for (auto& cap : video_captures_) {
        if (cap.isOpened()) {
            cap.release();
        }
    }
}

void VideoPlayerNode::getParams()
{
    // 声明和获取参数
    this->declare_parameter<std::string>("video_directory", "/home/shm-white/RM2026/videos");
    this->declare_parameter<double>("framerate", 30.0);
    this->declare_parameter<int>("video_width", 640);
    this->declare_parameter<int>("video_height", 480);
    this->declare_parameter<std::string>("camera_frame_id", "camera");
    this->declare_parameter<bool>("loop_videos", true);
    
    this->get_parameter("video_directory", video_directory_);
    this->get_parameter("framerate", framerate_);
    this->get_parameter("video_width", video_width_);
    this->get_parameter("video_height", video_height_);
    this->get_parameter("camera_frame_id", camera_frame_id_);
    this->get_parameter("loop_videos", loop_videos_);
    
    RCLCPP_INFO(this->get_logger(), "视频目录: %s", video_directory_.c_str());
    RCLCPP_INFO(this->get_logger(), "帧率: %.1f", framerate_);
    RCLCPP_INFO(this->get_logger(), "分辨率: %dx%d", video_width_, video_height_);
}

void VideoPlayerNode::setupPublishers()
{
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
}

void VideoPlayerNode::setupServices()
{
    param_service_ = this->create_service<camera_interfaces::srv::ParamEvent>(
        "param_event", 
        std::bind(&VideoPlayerNode::paramServiceCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
}

void VideoPlayerNode::paramServiceCallback(
    const std::shared_ptr<camera_interfaces::srv::ParamEvent::Request> request,
    std::shared_ptr<camera_interfaces::srv::ParamEvent::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "收到参数更改请求: %d", request->param_name);
    
    switch (request->param_name) {
        case camera_interfaces::srv::ParamEvent::Request::CAMERA_WIDTH:
            video_width_ = static_cast<int>(request->value);
            param_change = VIDEO_WIDTH;
            break;
        case camera_interfaces::srv::ParamEvent::Request::CAMERA_HEIGHT:
            video_height_ = static_cast<int>(request->value);
            param_change = VIDEO_HEIGHT;
            break;
        case camera_interfaces::srv::ParamEvent::Request::CAMERA_FRAMERATE:
            framerate_ = request->value;
            param_change = VIDEO_FRAMERATE;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "未知的参数类型: %d", request->param_name);
            break;
    }
    
    response->success = true;
    response->status_message = "参数更新成功";
    response->camera_width = video_width_;
    response->camera_height = video_height_;
    response->camera_framerate = static_cast<int>(framerate_);
}

bool VideoPlayerNode::init()
{
    // 创建视频目录（如果不存在）
    if (!std::filesystem::exists(video_directory_)) {
        RCLCPP_WARN(this->get_logger(), "视频目录不存在，正在创建: %s", video_directory_.c_str());
        try {
            std::filesystem::create_directories(video_directory_);
        } catch (const std::filesystem::filesystem_error& ex) {
            RCLCPP_ERROR(this->get_logger(), "创建视频目录失败: %s", ex.what());
            return false;
        }
    }
    
    // 加载视频文件
    if (!loadVideoFiles()) {
        RCLCPP_ERROR(this->get_logger(), "加载视频文件失败");
        return false;
    }
    
    initialized_ = true;
    return true;
}

bool VideoPlayerNode::loadVideoFiles()
{
    video_files_.clear();
    video_captures_.clear();
    
    // 扫描视频目录
    try {
        for (const auto& entry : std::filesystem::directory_iterator(video_directory_)) {
            if (entry.is_regular_file() && isVideoFile(entry.path().filename().string())) {
                video_files_.push_back(entry.path().string());
            }
        }
    } catch (const std::filesystem::filesystem_error& ex) {
        RCLCPP_ERROR(this->get_logger(), "扫描视频目录失败: %s", ex.what());
        return false;
    }
    
    if (video_files_.empty()) {
        RCLCPP_WARN(this->get_logger(), "在目录 %s 中未找到视频文件", video_directory_.c_str());
        
        // 创建一个测试用的黑色图像作为默认输出
        current_frame_ = cv::Mat::zeros(video_height_, video_width_, CV_8UC3);
        cv::putText(current_frame_, "No video files found", cv::Point(50, video_height_/2), 
                   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        cv::putText(current_frame_, "Directory: " + video_directory_, cv::Point(50, video_height_/2 + 40), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
        return true;  // 仍然返回true，使用默认图像
    }
    
    // 排序文件列表
    std::sort(video_files_.begin(), video_files_.end());
    
    RCLCPP_INFO(this->get_logger(), "找到 %zu 个视频文件:", video_files_.size());
    for (const auto& file : video_files_) {
        RCLCPP_INFO(this->get_logger(), "  %s", file.c_str());
    }
    
    // 初始化视频捕获器
    video_captures_.resize(video_files_.size());
    for (size_t i = 0; i < video_files_.size(); ++i) {
        video_captures_[i].open(video_files_[i]);
        if (!video_captures_[i].isOpened()) {
            RCLCPP_WARN(this->get_logger(), "无法打开视频文件: %s", video_files_[i].c_str());
        }
    }
    
    current_video_index_ = 0;
    return true;
}

bool VideoPlayerNode::isVideoFile(const std::string& filename) const
{
    std::string lower_filename = filename;
    std::transform(lower_filename.begin(), lower_filename.end(), lower_filename.begin(), ::tolower);
    
    const std::vector<std::string> video_extensions = {
        ".mp4", ".avi", ".mov", ".mkv", ".wmv", ".flv", ".webm", ".m4v"
    };
    
    for (const auto& ext : video_extensions) {
        if (lower_filename.length() >= ext.length() && 
            lower_filename.substr(lower_filename.length() - ext.length()) == ext) {
            return true;
        }
    }
    return false;
}

bool VideoPlayerNode::getNextFrame(cv::Mat& frame)
{
    if (video_files_.empty()) {
        frame = current_frame_.clone();
        return true;
    }
    
    if (current_video_index_ >= video_captures_.size()) {
        if (loop_videos_) {
            resetToFirstVideo();
        } else {
            return false;
        }
    }
    
    cv::VideoCapture& current_cap = video_captures_[current_video_index_];
    
    if (!current_cap.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "当前视频未打开，尝试重新打开");
        current_cap.open(video_files_[current_video_index_]);
        if (!current_cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频: %s", video_files_[current_video_index_].c_str());
            current_video_index_++;
            return getNextFrame(frame);  // 尝试下一个视频
        }
    }
    
    if (!current_cap.read(frame)) {
        // 当前视频播放完毕，切换到下一个
        RCLCPP_INFO(this->get_logger(), "视频 %s 播放完毕，切换到下一个", 
                   video_files_[current_video_index_].c_str());
        current_video_index_++;
        return getNextFrame(frame);  // 递归获取下一个视频的帧
    }
    
    // 调整帧大小
    if (frame.cols != video_width_ || frame.rows != video_height_) {
        cv::resize(frame, frame, cv::Size(video_width_, video_height_));
    }
    
    return true;
}

void VideoPlayerNode::resetToFirstVideo()
{
    current_video_index_ = 0;
    if (!video_files_.empty() && current_video_index_ < video_captures_.size()) {
        video_captures_[current_video_index_].set(cv::CAP_PROP_POS_FRAMES, 0);
    }
    RCLCPP_INFO(this->get_logger(), "重置到第一个视频");
}

sensor_msgs::msg::CameraInfo VideoPlayerNode::createCameraInfo()
{
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.frame_id = camera_frame_id_;
    camera_info.header.stamp = this->now();
    camera_info.width = video_width_;
    camera_info.height = video_height_;
    
    // 设置简单的相机矩阵（用于调试）
    camera_info.k[0] = video_width_;   // fx
    camera_info.k[4] = video_height_;  // fy
    camera_info.k[2] = video_width_ / 2.0;  // cx
    camera_info.k[5] = video_height_ / 2.0; // cy
    camera_info.k[8] = 1.0;
    
    return camera_info;
}

void VideoPlayerNode::publishImage()
{
    if (!initialized_) {
        return;
    }
    
    cv::Mat frame;
    if (!getNextFrame(frame)) {
        RCLCPP_ERROR(this->get_logger(), "获取视频帧失败");
        return;
    }
    
    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "获取到空帧");
        return;
    }
    
    // 转换为ROS图像消息
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = camera_frame_id_;
    
    sensor_msgs::msg::Image::SharedPtr image_msg = 
        cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    
    // 发布图像和相机信息
    image_publisher_->publish(*image_msg);
    camera_info_publisher_->publish(createCameraInfo());
}

void VideoPlayerNode::setVideoParams(VideoParams which_param)
{
    switch (which_param) {
        case ALL_PARAMS:
        case VIDEO_WIDTH:
        case VIDEO_HEIGHT:
            // 对于视频播放器，这些参数主要影响输出分辨率
            RCLCPP_INFO(this->get_logger(), "设置视频分辨率: %dx%d", video_width_, video_height_);
            break;
        case VIDEO_FRAMERATE:
            RCLCPP_INFO(this->get_logger(), "设置视频帧率: %.1f", framerate_);
            break;
        case NONE:
            break;
    }
}

void VideoPlayerNode::run()
{
    RCLCPP_INFO(this->get_logger(), "开始视频播放循环，帧率: %.1f fps", framerate_);
    
    rclcpp::WallRate loop_rate(framerate_);
    
    while (rclcpp::ok()) {
        publishImage();
        
        // 处理参数更改
        if (param_change != NONE) {
            setVideoParams(param_change);
            param_change = NONE;
        }
        
        rclcpp::spin_some(shared_from_this());
        loop_rate.sleep();
    }
}
