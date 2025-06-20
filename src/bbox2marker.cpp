#include "bbox2marker/bbox2marker.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

namespace bbox2marker
{
BBoxToMarker::BBoxToMarker()
: Node("bbox_to_marker_node")
{
    declare_parameter("cloud_width", 448);
    declare_parameter("cloud_height", 256);
    declare_parameter("image_width", 1920);
    declare_parameter("image_height", 1080);

    get_parameter("cloud_width", cloud_width_);
    get_parameter("cloud_height", cloud_height_);
    get_parameter("image_width", image_width_);
    get_parameter("image_height", image_height_);

    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "/cone_detections", 10,
        std::bind(&BBoxToMarker::detection_callback, this, std::placeholders::_1));

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", 10,
        std::bind(&BBoxToMarker::pointcloud_callback, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/cones", 10);
}

void BBoxToMarker::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    latest_detections_ = msg;
}

void BBoxToMarker::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!latest_detections_)
        return;

    latest_cloud_ = msg;

    float scale_x = static_cast<float>(cloud_width_) / image_width_;
    float scale_y = static_cast<float>(cloud_height_) / image_height_;

    visualization_msgs::msg::Marker marker_blue;     // use more markers to specify if a cone is blue(class 0), yellow(class 4) or others
    visualization_msgs::msg::Marker marker_yellow;     // use more markers to specify if a cone is blue(class 0), yellow(class 4) or others
    visualization_msgs::msg::Marker marker_other;     // use more markers to specify if a cone is blue(class 0), yellow(class 4) or others

    int id = 0;

    for (const auto &detection : latest_detections_->detections)
    {
        const auto &bbox = detection.bbox;
        int u = static_cast<int>(bbox.center.position.x * scale_x);
        int v = static_cast<int>(bbox.center.position.y * scale_y);   // return the vertical center of the bbox
        //int v = static_cast<int>(bbox.center.position.y * scale_y - bbox.size_y / 2);   // return the base of the bbox

        if (u < 0 || u >= cloud_width_ || v < 0 || v >= cloud_height_)
            continue;

        if (detection.results[0].hypothesis.class_id == "0")
        {
            marker_blue.header = msg->header;
            marker_blue.ns = "blue_cones";     // use more markers to specify if a cone is blue(class 0), yellow(class 4) or others
            marker_blue.id = ++id;
            marker_blue.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker_blue.action = visualization_msgs::msg::Marker::ADD;
            marker_blue.scale.x = 0.15;
            marker_blue.scale.y = 0.15;
            marker_blue.scale.z = 0.15;
            marker_blue.color.r = 0.0;
            marker_blue.color.g = 0.0;
            marker_blue.color.b = 1.0;
            marker_blue.color.a = 1.0;

            geometry_msgs::msg::Point point;
            if (get_point_from_uv(u, v, point))
            {
                marker_blue.points.push_back(point);
            }
        }else if (detection.results[0].hypothesis.class_id == "4")
        {
            marker_yellow.header = msg->header;
            marker_yellow.ns = "yellow_cones";     // use more markers to specify if a cone is blue(class 0), yellow(class 4) or others
            marker_yellow.id = ++id;
            marker_yellow.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker_yellow.action = visualization_msgs::msg::Marker::ADD;
            marker_yellow.scale.x = 0.15;
            marker_yellow.scale.y = 0.15;
            marker_yellow.scale.z = 0.15;
            marker_yellow.color.r = 0.5;
            marker_yellow.color.g = 0.5;
            marker_yellow.color.b = 0.0;
            marker_yellow.color.a = 1.0;

            geometry_msgs::msg::Point point;
            if (get_point_from_uv(u, v, point))
            {
                marker_yellow.points.push_back(point);
            }
        }else
        {
            marker_other.header = msg->header;
            marker_other.ns = "other_cones";     // use more markers to specify if a cone is blue(class 0), yellow(class 4) or others
            marker_other.id = ++id;
            marker_other.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker_other.action = visualization_msgs::msg::Marker::ADD;
            marker_other.scale.x = 0.15;
            marker_other.scale.y = 0.15;
            marker_other.scale.z = 0.15;
            marker_other.color.r = 1.0;
            marker_other.color.g = 1.0;
            marker_other.color.b = 1.0;
            marker_other.color.a = 1.0;

            geometry_msgs::msg::Point point;
            if (get_point_from_uv(u, v, point))
            {
                marker_other.points.push_back(point);
            }
        }
    }

    marker_pub_->publish(marker_blue);
    marker_pub_->publish(marker_yellow);
    marker_pub_->publish(marker_other);
    RCLCPP_INFO(this->get_logger(), "found %d markers", id); // debug
}

bool BBoxToMarker::get_point_from_uv(int u, int v, geometry_msgs::msg::Point &point)
{
    if (!latest_cloud_)
        return false;

    int index = v * cloud_width_ + u;
    if (index >= static_cast<int>(latest_cloud_->width * latest_cloud_->height))
        return false;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_cloud_, "z");

    iter_x += index;
    iter_y += index;
    iter_z += index;

    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        return false;

    point.x = *iter_x;
    point.y = *iter_y;
    point.z = *iter_z;
    return true;
}
}  // namespace bbox2marker
