#include "bbox2marray/bbox2marray.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

namespace bbox2marray
{
BBoxToMarkerArray::BBoxToMarkerArray()
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
        std::bind(&BBoxToMarkerArray::detection_callback, this, std::placeholders::_1));

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", 10,
        std::bind(&BBoxToMarkerArray::pointcloud_callback, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/bbox_markers", 10);
}

void BBoxToMarkerArray::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    latest_detections_ = msg;
}

void BBoxToMarkerArray::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!latest_detections_)
        return;

    latest_cloud_ = msg;

    float scale_x = static_cast<float>(cloud_width_) / image_width_;
    float scale_y = static_cast<float>(cloud_height_) / image_height_;

    visualization_msgs::msg::MarkerArray marker_array;

    int id = 0;

    for (const auto &detection : latest_detections_->detections)
    {
        const auto &bbox = detection.bbox;
        int u = static_cast<int>(bbox.center.position.x * scale_x);
        int v = static_cast<int>(bbox.center.position.y * scale_y);

        if (u < 0 || u >= cloud_width_ || v < 0 || v >= cloud_height_)
            continue;

        geometry_msgs::msg::Point point;
        if (get_point_from_uv(u, v, point))
        {
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "bbox_centers";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = point;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }
    }

    marker_pub_->publish(marker_array);
}

bool BBoxToMarkerArray::get_point_from_uv(int u, int v, geometry_msgs::msg::Point &point)
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
}  // namespace bbox2marray
