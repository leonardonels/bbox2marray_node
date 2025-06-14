#ifndef BBOX2MARRAY_HPP_
#define BBOX2MARRAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace bbox2marray
{
class BBoxToMarkerArray : public rclcpp::Node
{
public:
    BBoxToMarkerArray();

private:
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    bool get_point_from_uv(int u, int v, geometry_msgs::msg::Point &point);

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

    int cloud_width_;
    int cloud_height_;
    int image_width_;
    int image_height_;
};
}  // namespace bbox2marray

#endif  // BBOX2MARRAY_HPP_
