#include "bbox2marker/bbox2marker.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <cmath>

/*
TODO: IMPLEMENT PROPER SYNCRONISATION

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Declare subscribers
std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sub_;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;

// Define sync policy and synchronizer
typedef message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray,
    sensor_msgs::msg::Image> SyncPolicy;

std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

detection_sub_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, "/cone_detections");
depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/zed/zed_node/depth/depth_registered");

sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *detection_sub_, *depth_sub_);
sync_->registerCallback(std::bind(&BBoxToMarker::synced_callback, this, std::placeholders::_1, std::placeholders::_2));

void BBoxToMarker::synced_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr detection_msg,
    const sensor_msgs::msg::Image::SharedPtr depth_msg)
{
    // Now these messages are time-synchronized
    // Safe to process them together
}

*/

namespace bbox2marker
{

BBoxToMarker::BBoxToMarker()
: Node("bbox_to_marker_node"),
  camera_info_received_(false)
{
    declare_parameter("image_width", 1920);
    declare_parameter("image_height", 1080);

    get_parameter("image_width", image_width_);
    get_parameter("image_height", image_height_);

    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "/cone_detections", 10,
        std::bind(&BBoxToMarker::detection_callback, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/zed/zed_node/depth/depth_registered", 10,
        std::bind(&BBoxToMarker::depth_callback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/zed/zed_node/depth/camera_info", 10,
        std::bind(&BBoxToMarker::camera_info_callback, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/cones", 10);
}

void BBoxToMarker::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    latest_detections_ = msg;
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

void BBoxToMarker::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!camera_info_received_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for camera info...");
        return;
    }

    if (!latest_detections_)
        return;

    try
    {
        latest_depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        latest_depth_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    publish_markers(latest_depth_header_);
}

void BBoxToMarker::publish_markers(const std_msgs::msg::Header &header)
{
    rclcpp::Time depth_time(header.stamp);
    rclcpp::Time detection_time(latest_detections_->header.stamp);
    if (std::abs((depth_time - detection_time).seconds()) > 0.05)
    {
        RCLCPP_WARN(this->get_logger(), "Depth and detection not synchronized: delta=%.3f sec", (depth_time - detection_time).seconds());
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    //marker.header.frame_id = "map";
    marker.ns = "cones";
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.lifetime = rclcpp::Duration(0, 10);

    int id = 0;

    float scale_x = static_cast<float>(image_width_) / image_width_;  // actually 1.0, but keep for flexibility
    float scale_y = static_cast<float>(image_height_) / image_height_;

    for (const auto &detection : latest_detections_->detections)
    {
        const auto &bbox = detection.bbox;
        int u = static_cast<int>(bbox.center.position.x);
        // Use the base of the bbox (vertical bottom)
        int v = static_cast<int>(bbox.center.position.y + bbox.size_y / 2.0);

        // Check bounds
        if (u < 0 || u >= latest_depth_image_.cols || v < 0 || v >= latest_depth_image_.rows)
            continue;

        geometry_msgs::msg::Point point;
        if (get_point_from_uv(u, v, point))
        {
            marker.points.push_back(point);

            // Assign color based on class id
            std_msgs::msg::ColorRGBA color;
            if (!detection.results.empty())
            {
                const std::string& class_id = detection.results[0].hypothesis.class_id;
                if (class_id == "0") // Blue cone
                {
                    color.r = 0.0f;
                    color.g = 0.0f;
                    color.b = 1.0f;
                    color.a = 1.0f;
                }
                else if (class_id == "4") // Yellow cone
                {
                    color.r = 1.0f;
                    color.g = 1.0f;
                    color.b = 0.0f;
                    color.a = 1.0f;
                }
                else // Other cones
                {
                    color.r = 1.0f;
                    color.g = 0.5f;
                    color.b = 0.0f;
                    color.a = 1.0f;
                }
            }
            else
            {
                // Default color if no class info
                color.r = 1.0f;
                color.g = 1.0f;
                color.b = 1.0f;
                color.a = 1.0f;
            }
            marker.colors.push_back(color);

            ++id;
        }
    }

    marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published %d markers", id);

    latest_detections_.reset();
}

bool BBoxToMarker::get_point_from_uv(int u, int v, geometry_msgs::msg::Point &point)
{
    if (latest_depth_image_.empty())
        return false;

    if (u < 0 || u >= latest_depth_image_.cols || v < 0 || v >= latest_depth_image_.rows)
        return false;

    float Z = latest_depth_image_.at<float>(v, u);
    if (std::isnan(Z) || Z <= 0.001f)
        return false;

    // Project pixel to camera frame 3D point
    point.x = (u - cx_) * Z / fx_;
    point.y = (v - cy_) * Z / fy_;
    point.z = Z;

    return true;
}

}  // namespace bbox2marker
