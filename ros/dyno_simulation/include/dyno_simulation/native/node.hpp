/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( D ) ( Y ) ( N ) ( O )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +              DYNO: Ground Vehicle Dynamics Validation Toolkit             +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <dyno/simulation/autonomous_navigation.hpp>
#include <nlohmann/json.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <dyno_interfaces/msg/obstacle_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

namespace DYNO {
namespace ROS {
namespace Native {

class NativeNode : public rclcpp::Node {
   public:
    NativeNode();

   protected:
    void Configure();
    void GetParameters();
    void Initialize();
    void Activate();
    void CreatePublishers();
    void CreateSubscriptions();
    void CreateTimers();
    void StartTimers();

   private:
    void InitializeSimulation();

    std::shared_ptr<DYNO::Simulation::AutonomousNavigation> simulation_;
    std::shared_ptr<chrono::ChBodyAuxRef> chassis_body_;
    std::thread simulation_thread_;

    void SimulationTask();

    // Simulation
    // ----------

    void Publish();

    // ----------

    // Controllers
    // -----------

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        ackermann_subscription_;

    void AckermannCallback(
        ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr
            ackermann_message);

    void AdvanceControllers();

    double control_period_;

    rclcpp::Time last_control_time_;
    // -----------

    // Clock
    // -----

    /** @brief Shared pointer to the clock publisher. */
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

    void PublishClock();
    // -----

    // Transforms
    // ----------

    double transforms_period_;

    rclcpp::Time last_transforms_time_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    void PublishTransforms();

    std::shared_ptr<geometry_msgs::msg::TransformStamped>
    GetTransformStampedMessage(chrono::ChFrame<double> frame,
                               rclcpp::Time stamp, std::string frame_id,
                               std::string child_frame_id);
    // ----------

    // Joint states
    // ------------

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publisher_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    void PublishPath();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        target_pose_publisher_;

    void PublishTargetPose();

    rclcpp::Publisher<dyno_interfaces::msg::ObstacleArray>::SharedPtr
        obstacles_publisher_;

    void PublishObstacles();

    // GNSS/INS odometry
    // -----------------

    /** @brief Shared pointer to the odometry publisher. */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    void PublishOdometry();

    double odometry_period_;

    rclcpp::Time last_odometry_time_;
    // -----------------

    /** @brief Shared pointer to the steering angle publisher. */
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        steering_angle_publisher_;

    double steering_angle_ = 0.0;

    void PublishSteeringAngle();

    /** @brief Shared pointer to the IMU publisher. */
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    void PublishImu();

    double imu_period_;

    rclcpp::Time last_imu_time_;

    // LiDAR point cloud
    // -----------------

    bool publish_pointcloud_ = false;

    /** @brief Shared pointer to the point cloud publisher. */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointcloud_publisher_;

    /** @brief Publish the latest received point cloud. */
    void PublishPointCloud();

    /** @brief Rate at which point clouds are published. */
    double pointcloud_period_;

    /** @brief Last time stamp for a received point cloud. */
    rclcpp::Time last_pointcloud_time_;
    // -----------------

    // Camera image
    // ------------

    bool publish_camera_ = false;

    /** @brief Shared pointer to the image publisher. */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    void PublishImage();

    /** @brief Shared pointer to the image publisher. */
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_publisher_;

    void PublishCameraInfo();

    double image_period_;

    rclcpp::Time last_image_time_;
    // ------------

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        throttle_effort_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        brake_effort_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        steering_effort_publisher_;

    void PublishCommands();

    bool CheckElapsedTime(const rclcpp::Time& current_time,
                          rclcpp::Time& last_time, const double& period);

    std::shared_ptr<DYNO::Interfaces::JSONConfiguration> configuration_;
};

}  // namespace Native
}  // namespace ROS
}  // namespace DYNO
