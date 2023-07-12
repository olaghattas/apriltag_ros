//// Copyright 2019 Intelligent Robotics Lab
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.
//
//
#include <memory>
#include <string>
#include <map>
#include <vector>
//

#include "rclcpp/rclcpp.hpp"

#include <shr_aptag_parameters.hpp>
#include <std_msgs/msg/bool.hpp>
//
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <stdio.h>
#include <math.h>
#include <opencv2/opencv.hpp>
// Created by ola on 7/12/23.
//

inline float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

inline float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

// quaternion = [w, x, y, z]'
cv::Mat mRot2Quat(const cv::Mat& m) {
    float r11 = m.at<float>(0, 0);
    float r12 = m.at<float>(0, 1);
    float r13 = m.at<float>(0, 2);
    float r21 = m.at<float>(1, 0);
    float r22 = m.at<float>(1, 1);
    float r23 = m.at<float>(1, 2);
    float r31 = m.at<float>(2, 0);
    float r32 = m.at<float>(2, 1);
    float r33 = m.at<float>(2, 2);
    float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
    float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
    float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
    if (q0 < 0.0f) {
        q0 = 0.0f;
    }
    if (q1 < 0.0f) {
        q1 = 0.0f;
    }
    if (q2 < 0.0f) {
        q2 = 0.0f;
    }
    if (q3 < 0.0f) {
        q3 = 0.0f;
    }
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);
    if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
        q0 *= +1.0f;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    }
    else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0f;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    }
    else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0f;
        q3 *= SIGN(r32 + r23);
    }
    else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0f;
    }
    else {
        printf("coding error\n");
    }
    float r = NORM(q0, q1, q2, q3);
    q0 /= r;
    q1 /= r;
    q2 /= r;
    q3 /= r;

    cv::Mat res = (cv::Mat_<float>(4, 1) << q0, q1, q2, q3);
    return res;
}

cv::Mat string2Matrix(std::string rotationString) {
//"[1,0,0][0,1,0][0,0,1]"

// Remove the square brackets
    rotationString.erase(std::remove(rotationString.begin(), rotationString.end(), '['), rotationString.end());
    rotationString.erase(std::remove(rotationString.begin(), rotationString.end(), ']'), rotationString.end());

// Parse the string values and create the rotation matrix
    cv::Mat rotationMatrix(3, 3, CV_32F);
    sscanf(rotationString.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f",
           &rotationMatrix.at<float>(0, 0), &rotationMatrix.at<float>(0, 1), &rotationMatrix.at<float>(0, 2),
           &rotationMatrix.at<float>(1, 0), &rotationMatrix.at<float>(1, 1), &rotationMatrix.at<float>(1, 2),
           &rotationMatrix.at<float>(2, 0), &rotationMatrix.at<float>(2, 1), &rotationMatrix.at<float>(2, 2));

// Print the resulting rotation matrix
    std::cout << "Rotation Matrix:" << std::endl;
    std::cout << rotationMatrix << std::endl;
    return rotationMatrix;
}

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher(const std::string id,  double pos_x, double pos_y, cv::Mat quat)
            : Node("apatg_frame_publisher"+ id) {
//          Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        clock_ = rclcpp::Clock(rcl_clock_type_e::RCL_ROS_TIME);
        auto func = [this]() -> void { timer_callback(); };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), func);
        id_ = id;
        pos_x_ = pos_x;
        pos_y_ = pos_y;
        quat_ = quat;

    }

    void timer_callback() {
        geometry_msgs::msg::TransformStamped t;

        // Fill in the message
        t.header.frame_id = "map";
        t.child_frame_id = id_;

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = pos_x_;
        t.transform.translation.y = pos_y_;
        t.transform.translation.z = 0.15;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        t.transform.rotation.x = quat_.at<float>(0);
        t.transform.rotation.y = quat_.at<float>(1);
        t.transform.rotation.z = quat_.at<float>(2);
        t.transform.rotation.w = quat_.at<float>(3);

        t.header.stamp = clock_.now();
        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string id_;
    double pos_x_;
    double pos_y_;
    cv::Mat quat_ = quat;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;


    auto n = std::make_shared<rclcpp::Node>("aptag_params");
    auto param_listener = std::make_shared<shr_aptag_parameters::ParamListener>(n);
    auto params = param_listener->get_params();

    std::vector<std::shared_ptr<FramePublisher>> nodes;
    for (auto i = 0ul ; i < params.aptags_world_frames.aptags.size(); i++){
        auto id = params.aptags_world_frames.aptags[i];
        auto pos_x = params.aptags_world_frames.position_x[i];
        auto pos_y = params.aptags_world_frames.position_y[i];
        auto rotation_matrix = params.aptags_world_frames.rotation_matrix[i];

        cv::Mat rotmatrix = string2Matrix(rotation_matrix);
        cv::Mat quat = mRot2Quat(rotmatrix); //[w,x,y,z]


        auto node = std::make_shared<FramePublisher>(id, pos_x, pos_y, quat);
        auto listener = std::make_shared<shr_aptag_parameters::ParamListener>(node);
        exe.add_node(node);
        nodes.push_back(node);
    }

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
