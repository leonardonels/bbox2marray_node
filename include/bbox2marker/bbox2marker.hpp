#ifndef BBOX2MARKER_HPP_
#define BBOX2MARKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <opencv2/opencv.hpp>

// TF2 and message filters
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace bbox2marker
{

class BBoxToMarker : public rclcpp::Node
{
public:
    BBoxToMarker();

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void synced_callback(
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
    
    void publish_markers(const std_msgs::msg::Header &header);
    bool get_point_from_uv(int u, int v, geometry_msgs::msg::Point &point_out);

    // Message filter subscribers
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;

    // Sync
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        vision_msgs::msg::Detection2DArray,
        sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Camera info subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Latest messages
    vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_detections_;
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
