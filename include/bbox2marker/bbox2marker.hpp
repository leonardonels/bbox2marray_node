#ifndef BBOX2MARKER_HPP_
#define BBOX2MARKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <opencv2/opencv.hpp>

namespace bbox2marker
{

class BBoxToMarker : public rclcpp::Node
{
public:
    BBoxToMarker();

private:
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void publish_markers(const std_msgs::msg::Header &header);
    bool get_point_from_uv(int u, int v, geometry_msgs::msg::Point &point);

    // Subscribers
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Latest messages
    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    cv::Mat latest_depth_image_;
    std_msgs::msg::Header latest_depth_header_;

    // Camera intrinsics
    bool camera_info_received_;
    float fx_, fy_, cx_, cy_;

    // Parameters
    int image_width_;
    int image_height_;
};

}  // namespace bbox2marker

#endif  // BBOX2MARKER_HPP_
