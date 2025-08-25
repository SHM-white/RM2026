#include "video_player/video_player.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VideoPlayerNode>("video_player_node");
    
    try {
        node->run();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "视频播放器异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
