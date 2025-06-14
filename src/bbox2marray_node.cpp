#include "bbox2marray/bbox2marray.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bbox2marray::BBoxToMarkerArray>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
