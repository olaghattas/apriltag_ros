#pragma once
// C Headers
#include <apriltag.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
// C++ Headers
#include <map>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// apriltag
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "common/homography.h"


class AprilTagNode : public rclcpp::Node {
public:
    explicit AprilTagNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~AprilTagNode();

private:
    static int idComparison(const void *first, const void *second);
    void removeDuplicates(zarray_t *detections_);
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img);

    void addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d> &objectPoints) const;
    void addImagePoints(apriltag_detection_t *detection, std::vector<cv::Point2d> &imagePoints) const;
    Eigen::Matrix4d getRelativeTransform(
            std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints, double fx, double fy,
            double cx, double cy) const;
    geometry_msgs::msg::TransformStamped makeTagPose(
            const Eigen::Matrix4d &transform, const Eigen::Quaternion<double> rot_quaternion,
            const std_msgs::msg::Header &header);

    image_transport::Subscriber sub_cam;
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    rclcpp::Clock clock_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    apriltag_detector_t *td;
    static const std::map<std::string, apriltag_family_t * (*)(void)> tag_create;
    static const std::map<std::string, void (*)(apriltag_family_t *)> tag_destroy;

    std::string tag_family;
    double tag_edge_size;
    int max_hamming;
    bool z_up;
    bool remove_duplicates_;
    double camera_fx;
    double camera_fy;
    double camera_cx;
    double camera_cy;


    std::map<int, std::string> tag_frames;
    std::map<int, double> tag_sizes;
    apriltag_family_t *tf;

    rclcpp::TimerBase::SharedPtr timer_;
};

