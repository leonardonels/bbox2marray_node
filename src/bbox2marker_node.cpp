#include "bbox2marker/bbox2marker.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bbox2marker::BBoxToMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
