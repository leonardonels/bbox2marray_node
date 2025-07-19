#include "bbox2marker/bbox2marker.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <cmath>

namespace bbox2marker
{

BBoxToMarker::BBoxToMarker()
: Node("bbox_to_marker_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  camera_info_received_(false)
{
    declare_parameter("image_width", 1920);
    declare_parameter("image_height", 1080);

    get_parameter("image_width", image_width_);
    get_parameter("image_height", image_height_);

    // TF and message filter-based subscribers
    detection_sub_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, "/cone_detections");
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/zed/zed_node/depth/depth_registered");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *detection_sub_, *depth_sub_);
    sync_->registerCallback(std::bind(&BBoxToMarker::synced_callback, this, std::placeholders::_1, std::placeholders::_2));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/zed/zed_node/depth/camera_info", 10,
        std::bind(&BBoxToMarker::camera_info_callback, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/cones", 10);
}

void BBoxToMarker::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (!camera_info_received_)
    {
        fx_ = static_cast<float>(msg->k[0]);
        fy_ = static_cast<float>(msg->k[4]);
        cx_ = static_cast<float>(msg->k[2]);
        cy_ = static_cast<float>(msg->k[5]);
        camera_info_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Camera intrinsics received: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
    }
}

void BBoxToMarker::synced_callback(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr depth_msg)

{
    if (!camera_info_received_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for camera info...");
        return;
    }

    try
    {
        latest_depth_image_ = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        latest_depth_header_ = depth_msg->header;
        latest_detections_ = detection_msg;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    publish_markers(depth_msg->header);
}

void BBoxToMarker::publish_markers(const std_msgs::msg::Header &header)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = header.stamp;
    marker.ns = "cones";
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.lifetime = rclcpp::Duration(0, 10);

    int id = 0;

    for (const auto &detection : latest_detections_->detections)
    {
        const auto &bbox = detection.bbox;
        int u = static_cast<int>(bbox.center.position.x);
        int v = static_cast<int>(bbox.center.position.y + bbox.size_y / 2.0);

        if (u < 0 || u >= latest_depth_image_.cols || v < 0 || v >= latest_depth_image_.rows)
            continue;

        geometry_msgs::msg::Point point;
        if (get_point_from_uv(u, v, point))
        {
            marker.points.push_back(point);

            std_msgs::msg::ColorRGBA color;
            if (!detection.results.empty())
            {
                const std::string& class_id = detection.results[0].hypothesis.class_id;
                if (class_id == "0") // Blue
                {
                    color.r = 0.0f;
                    color.g = 0.0f;
                    color.b = 1.0f;
                }
                else if (class_id == "4") // Yellow
                {
                    color.r = 1.0f;
                    color.g = 1.0f;
                    color.b = 0.0f;
                }
                else // Others
                {
                    color.r = 1.0f;
                    color.g = 0.5f;
                    color.b = 0.0f;
                }
                color.a = 1.0f;
            }
            else
            {
                color.r = color.g = color.b = color.a = 1.0f;
            }

            marker.colors.push_back(color);
            ++id;
        }
    }

    marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published %d markers", id);

    latest_detections_.reset();
}

bool BBoxToMarker::get_point_from_uv(int u, int v, geometry_msgs::msg::Point &point_out)
{
    if (latest_depth_image_.empty())
        return false;

    if (u < 0 || u >= latest_depth_image_.cols || v < 0 || v >= latest_depth_image_.rows)
        return false;

    float Z = latest_depth_image_.at<float>(v, u);
    if (std::isnan(Z) || Z <= 0.001f)
        return false;

    geometry_msgs::msg::PointStamped point_cam;
    point_cam.header.frame_id = latest_depth_header_.frame_id;
    point_cam.header.stamp = latest_depth_header_.stamp;
    point_cam.point.x = (u - cx_) * Z / fx_;
    point_cam.point.y = (v - cy_) * Z / fy_;
    point_cam.point.z = Z;

    try
    {
        auto point_map = tf_buffer_.transform(point_cam, "map", tf2::durationFromSec(0.05));
        point_out = point_map.point;
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
        return false;
    }
}

}  // namespace bbox2marker
