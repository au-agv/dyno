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

#include <dyno_simulation/native/node.hpp>

namespace DYNO {
namespace ROS {
namespace Native {

NativeNode::NativeNode() : rclcpp::Node("native") {
    Configure();
    Activate();
}

void NativeNode::Configure() {
    GetParameters();
    Initialize();
}

void NativeNode::GetParameters() {
    declare_parameter("options", "");

    declare_parameter("rates.transforms", 1000.0);
    transforms_period_ = 1.0 / get_parameter("rates.transforms").as_double();

    declare_parameter("rates.odometry", 100.0);
    odometry_period_ = 1.0 / get_parameter("rates.odometry").as_double();

    declare_parameter("rates.imu", 100.0);
    imu_period_ = 1.0 / get_parameter("rates.imu").as_double();

    declare_parameter("sensors.lidar", false);
    publish_pointcloud_ = get_parameter("sensors.lidar").as_bool();

    declare_parameter("rates.pointcloud", 10.0);
    pointcloud_period_ = 1.0 / get_parameter("rates.pointcloud").as_double();

    declare_parameter("sensors.camera", false);
    publish_camera_ = get_parameter("sensors.camera").as_bool();

    declare_parameter("rates.image", 15.0);
    image_period_ = 1.0 / get_parameter("rates.image").as_double();

    declare_parameter("rates.controllers", 200.0);
    control_period_ = 1.0 / get_parameter("rates.controllers").as_double();
}

void NativeNode::Initialize() {
    // Initialize the DYNO simulation.
    InitializeSimulation();

    // Initialize the timestamps for the message publishing tasks. This will
    // ensure the messages are first published after one period (e.g. for a 1 Hz
    // publish rate, the first message will be published one second after the
    // simulation start, in simulation time).
    auto current_time = get_clock()->now();
    last_transforms_time_ = current_time;
    last_control_time_ = current_time;
    last_odometry_time_ = current_time;
    last_imu_time_ = current_time;
    last_pointcloud_time_ = current_time;
    last_image_time_ = current_time;
}

void NativeNode::Activate() {
    CreateSubscriptions();
    CreatePublishers();
    StartTimers();
}

void NativeNode::CreatePublishers() {
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        "sensors/navigation/odometry",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "sensors/navigation/imu", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "sensors/lidar/points", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "sensors/camera/mono/image/raw",
        RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "sensors/camera/mono/info", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    steering_angle_publisher_ = create_publisher<std_msgs::msg::Float64>(
        "sensors/steering/angle", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    throttle_effort_publisher_ = create_publisher<std_msgs::msg::Float64>(
        "controls/throttle", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    brake_effort_publisher_ = create_publisher<std_msgs::msg::Float64>(
        "controls/brake", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    steering_effort_publisher_ = create_publisher<std_msgs::msg::Float64>(
        "controls/steering", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "model/joints/steering", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "navigation/path",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    waypoints_publisher_ = create_publisher<nav_msgs::msg::Path>(
        "navigation/waypoints",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    target_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "navigation/target",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    status_publisher_ = create_publisher<dyno_interfaces::msg::Status>(
        "simulator/status",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    obstacles_publisher_ =
        create_publisher<dyno_interfaces::msg::ObstacleArray>(
            "scenario/obstacles",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    transform_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void NativeNode::PublishPath() {
    auto stamp = get_clock()->now();
    auto path = simulation_->GetNavigationPath();
    nav_msgs::msg::Path path_message;
    path_message.header.frame_id = "map";
    path_message.header.stamp = stamp;
    for (const auto& position : path) {
        geometry_msgs::msg::PoseStamped pose_message;
        pose_message.header.frame_id = "map";
        pose_message.header.stamp = stamp;
        pose_message.pose.position.x = position.x();
        pose_message.pose.position.y = position.y();
        pose_message.pose.position.z = position.z();
        path_message.poses.push_back(pose_message);
    }
    path_publisher_->publish(path_message);
}

void NativeNode::PublishWaypoints() {
    auto stamp = get_clock()->now();
    auto path = simulation_->GetNavigationWaypoints();
    nav_msgs::msg::Path path_message;
    path_message.header.frame_id = "map";
    path_message.header.stamp = stamp;
    for (const auto& position : path) {
        geometry_msgs::msg::PoseStamped pose_message;
        pose_message.header.frame_id = "map";
        pose_message.header.stamp = stamp;
        pose_message.pose.position.x = position.x();
        pose_message.pose.position.y = position.y();
        pose_message.pose.position.z = position.z();
        path_message.poses.push_back(pose_message);
    }
    waypoints_publisher_->publish(path_message);
}

void NativeNode::PublishTargetPose() {
    auto stamp = get_clock()->now();

    // TODO: This message could also implement the heading angle for planners
    // that make use of that information in target poses.
    auto target = simulation_->GetTargetWaypoint();
    geometry_msgs::msg::PoseStamped pose_message;
    pose_message.header.frame_id = "map";
    pose_message.header.stamp = stamp;
    pose_message.pose.position.x = target.x();
    pose_message.pose.position.y = target.y();
    pose_message.pose.position.z = target.z();
    target_pose_publisher_->publish(pose_message);
}

void NativeNode::PublishObstacles() {
    auto stamp = get_clock()->now();
    auto obstacles = simulation_->GetObstacles();
    dyno_interfaces::msg::ObstacleArray obstacles_message;
    for (const auto& obstacle : obstacles) {
        dyno_interfaces::msg::Obstacle obstacle_message;
        obstacle_message.header.frame_id = "map";
        obstacle_message.header.stamp = stamp;
        obstacle_message.pose.pose.position.x = obstacle.GetPosition().x();
        obstacle_message.pose.pose.position.y = obstacle.GetPosition().y();
        obstacle_message.pose.pose.position.z = obstacle.GetPosition().z();
        obstacle_message.size.x = obstacle.GetSize().x();
        obstacle_message.size.y = obstacle.GetSize().y();
        obstacle_message.size.z = obstacle.GetSize().z();
        obstacles_message.obstacles.push_back(obstacle_message);
    }
    obstacles_publisher_->publish(obstacles_message);
}

void NativeNode::CreateSubscriptions() {
    ackermann_subscription_ =
        create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "controls/drive", 1,
            std::bind(&NativeNode::AckermannCallback, this,
                      std::placeholders::_1));
}

void NativeNode::StartTimers() {
    RCLCPP_DEBUG(get_logger(), "Spinning simulation thread...");

    // Spin a detached simulation thread.
    simulation_thread_ = std::thread(&NativeNode::SimulationTask, this);
    simulation_thread_.detach();
}

void NativeNode::InitializeSimulation() {
    // Load configuration file.
    auto options_file_path = get_parameter("options").as_string();
    configuration_ = std::make_shared<DYNO::Interfaces::JSONConfiguration>(
        options_file_path);
    const std::string arguments =
        "--scenario autonomousNavigation --configuration " + options_file_path;
    configuration_->ParseString(arguments);
    configuration_->ReadConfiguration();
}

void NativeNode::SimulationTask() {
    // Instantiate a new DYNO simulation and initialize it with the loaded JSON
    // options file.
    simulation_ = std::make_shared<DYNO::Simulation::AutonomousNavigation>(
        configuration_);
    simulation_->Initialize();  // <- This method MUST be called after loading
                                // the JSON options.

    // Store a shared pointer to the vehicle chassis body to obtain ground truth
    // kinematics variables values.
    chassis_body_ = simulation_->GetVehicle()->GetVehicle()->GetChassisBody();

    PublishObstacles();
    PublishPath();
    PublishWaypoints();
    PublishTargetPose();
    PublishStatus();

    while (rclcpp::ok()) {
        simulation_->Step();
        PublishClock();
        Publish();

        if (simulation_->IsCompleted()) {
            RCLCPP_ERROR(get_logger(), "Shutting down node ...");
            PublishStatus();
            rclcpp::shutdown();
            return;
        }
    }

    PublishStatus();
}

void NativeNode::PublishStatus() {
    dyno_interfaces::msg::Status message;
    message.successful = simulation_->IsSuccessful();
    message.completed = simulation_->IsCompleted();
    message.reason = simulation_->GetCompletionMessage();
    status_publisher_->publish(message);
}

void NativeNode::Publish() {
    auto current_time = get_clock()->now();

    if (CheckElapsedTime(current_time, last_transforms_time_,
                         transforms_period_)) {
        PublishTransforms();
    }

    if (CheckElapsedTime(current_time, last_odometry_time_, odometry_period_)) {
        PublishOdometry();
        steering_angle_ =
            simulation_->GetVehicle()->GetSteeringAngle() * 180.0 / M_PI;
        PublishSteeringAngle();
    }

    if (CheckElapsedTime(current_time, last_imu_time_, imu_period_)) {
        PublishImu();
    }

    if (publish_pointcloud_ &&
        CheckElapsedTime(current_time, last_pointcloud_time_,
                         pointcloud_period_)) {
        PublishPointCloud();
    }

    if (publish_camera_ &&
        CheckElapsedTime(current_time, last_image_time_, image_period_)) {
        PublishImage();
        PublishCameraInfo();
    }
}

void NativeNode::PublishClock() {
    // Get the current simulation time.
    auto simulation_time = simulation_->GetTime();

    // Split the time in seconds and nanoseconds since last second.
    int32_t seconds = std::floor(simulation_time);
    uint32_t nanoseconds = static_cast<uint32_t>(
        (simulation_time - std::floor(simulation_time)) * 1.0e9);

    RCLCPP_DEBUG(get_logger(), "Publishing simulated clock \"(%i s, %i ns)\"",
                 seconds, nanoseconds);

    // Publish the rosgraph_msgs/msg/Clock message.
    auto clock_message = std::make_shared<rosgraph_msgs::msg::Clock>();
    clock_message->clock.sec = seconds;
    clock_message->clock.nanosec = nanoseconds;
    clock_publisher_->publish(*clock_message);
}

void NativeNode::PublishTransforms() {
    auto chassis_frame = simulation_->GetVehicle()
                             ->GetVehicle()
                             ->GetChassisBody()
                             ->GetFrameRefToAbs();
    auto ins_frame = *(simulation_->GetIMUFrame());

    auto transform_message = GetTransformStampedMessage(
        chrono::ChFrame<double>(chrono::ChVector3<double>(0.0, 0.0, 0.0),
                                chrono::QUNIT),
        get_clock()->now(), "map", "odom");
    transform_broadcaster_->sendTransform(*transform_message);

    auto transform_message_odom = GetTransformStampedMessage(
        chrono::ChFrame<double>(chassis_frame.GetPos() + ins_frame.GetPos(),
                                chassis_frame.GetRot()),
        get_clock()->now(), "odom", "base_link");
    transform_broadcaster_->sendTransform(*transform_message_odom);

    /*
    // LiDAR transform
    auto base_link_to_lidar_transform_message = GetTransformStampedMessage(
        chrono::ChFrame<double>(
            simulation_->GetVehicle()->GetLiDARFrame()->GetPos(),
            simulation_->GetVehicle()->GetLiDARFrame()->GetRot()),
        get_clock()->now(),
        "base_link",
        "lidar_link");
    transform_broadcaster_->sendTransform(
        *base_link_to_lidar_transform_message);

    // Camera transform
    auto base_link_to_camera_transform_message = GetTransformStampedMessage(
        chrono::ChFrame<double>(
            simulation_->GetVehicle()->GetCameraFrame()->GetPos(),
            simulation_->GetVehicle()->GetCameraFrame()->GetRot() *
                chrono::Q_ROTATE_Z_TO_Y * chrono::Q_ROTATE_Z_TO_X),
        get_clock()->now(),
        "base_link",
        "camera_link");
    transform_broadcaster_->sendTransform(
        *base_link_to_camera_transform_message);
    */
}

std::shared_ptr<geometry_msgs::msg::TransformStamped>
NativeNode::GetTransformStampedMessage(chrono::ChFrame<double> frame,
                                       rclcpp::Time stamp, std::string frame_id,
                                       std::string child_frame_id) {
    auto transform_message =
        std::make_shared<geometry_msgs::msg::TransformStamped>();
    transform_message->header.stamp = stamp;
    transform_message->header.frame_id = frame_id;
    transform_message->child_frame_id = child_frame_id;

    // Populate the transform entries of the geometry_msgs/TransformStamped
    // message.
    transform_message->transform.translation.x = frame.GetPos().x();
    transform_message->transform.translation.y = frame.GetPos().y();
    transform_message->transform.translation.z = frame.GetPos().z();
    transform_message->transform.rotation.x = frame.GetRot().e1();
    transform_message->transform.rotation.y = frame.GetRot().e2();
    transform_message->transform.rotation.z = frame.GetRot().e3();
    transform_message->transform.rotation.w = frame.GetRot().e0();

    return transform_message;
}

void NativeNode::PublishOdometry() {
    nav_msgs::msg::Odometry odometry_message;

    auto chassis_frame = simulation_->GetVehicle()
                             ->GetVehicle()
                             ->GetChassisBody()
                             ->GetFrameRefToAbs();
    auto ins_frame = *(simulation_->GetIMUFrame());
    auto actual_frame = chassis_frame;

    odometry_message.header.stamp = get_clock()->now();
    odometry_message.header.frame_id = "odom";
    odometry_message.child_frame_id = "base_link";

    auto body = simulation_->GetVehicle()->GetVehicle()->GetChassisBody();
    auto position = body->GetPos() + ins_frame.GetPos();
    odometry_message.pose.pose.position.x = position.x();
    odometry_message.pose.pose.position.y = position.y();
    odometry_message.pose.pose.position.z = position.z();

    auto orientation = body->GetRot();
    odometry_message.pose.pose.orientation.w = orientation.e0();
    odometry_message.pose.pose.orientation.x = orientation.e1();
    odometry_message.pose.pose.orientation.y = orientation.e2();
    odometry_message.pose.pose.orientation.z = orientation.e3();

    auto linear_velocity = body->GetPosDt();
    auto angular_velocity = body->GetRotDt().GetCardanAnglesXYZ();
    auto rotation = body->GetRotMat();
    odometry_message.twist.twist.linear.x =
        Vdot(linear_velocity, rotation.GetAxisX());
    odometry_message.twist.twist.linear.y =
        Vdot(linear_velocity, rotation.GetAxisY());
    odometry_message.twist.twist.linear.z =
        Vdot(linear_velocity, rotation.GetAxisZ());
    odometry_message.twist.twist.angular.x = angular_velocity.x();
    odometry_message.twist.twist.angular.y = angular_velocity.y();
    odometry_message.twist.twist.angular.z = angular_velocity.z();

    odometry_publisher_->publish(odometry_message);
}

void NativeNode::PublishSteeringAngle() {

    auto steering_angle_message = std::make_shared<std_msgs::msg::Float64>();
    steering_angle_message->data = steering_angle_;
    steering_angle_publisher_->publish(*steering_angle_message);
}

void NativeNode::PublishImu() {}

void NativeNode::PublishPointCloud() {
    chrono::sensor::UserXYZIBufferPtr pcl_buffer =
        simulation_->GetLiDARSensor()
            ->GetSensor()
            ->GetMostRecentBuffer<chrono::sensor::UserXYZIBufferPtr>();
    size_t pcl_buffer_size = pcl_buffer->Height * pcl_buffer->Width;
    size_t pcl_buffer_step = 16;
    std::vector<uint8_t> pcl_data(pcl_buffer_step * pcl_buffer_size);

    if (pcl_buffer->Buffer) {
        for (size_t i = 0; i < pcl_buffer_size; ++i) {
            std::memcpy(&pcl_data[pcl_buffer_step * i],
                        &pcl_buffer->Buffer[i].x, sizeof(float));
            std::memcpy(&pcl_data[pcl_buffer_step * i + 4],
                        &pcl_buffer->Buffer[i].y, sizeof(float));
            std::memcpy(&pcl_data[pcl_buffer_step * i + 8],
                        &pcl_buffer->Buffer[i].z, sizeof(float));
            std::memcpy(&pcl_data[pcl_buffer_step * i + 12],
                        &pcl_buffer->Buffer[i].intensity, sizeof(float));
        }

        auto pointcloud_message =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        pointcloud_message->header.stamp = get_clock()->now();
        pointcloud_message->header.frame_id = "lidar_link";
        pointcloud_message->height = pcl_buffer->Height;
        pointcloud_message->width = pcl_buffer->Width;
        auto fields = std::vector<std::string>{"x", "y", "z", "intensity"};
        for (size_t i = 0; i < fields.size(); ++i) {
            auto pointfield_message =
                std::make_shared<sensor_msgs::msg::PointField>();
            pointfield_message->name = fields[i];
            pointfield_message->offset = 4 * i;
            pointfield_message->datatype = 7;
            pointfield_message->count = 1;
            pointcloud_message->fields.push_back(*pointfield_message);
        }
        pointcloud_message->is_bigendian = true;
        pointcloud_message->point_step = 16;
        pointcloud_message->row_step = 1;
        pointcloud_message->data = pcl_data;
        pointcloud_message->is_dense = false;
        pointcloud_publisher_->publish(*pointcloud_message);
    } else {
        RCLCPP_ERROR(get_logger(),
                     "LiDAR buffer not available during serialization.");
    }
}

void NativeNode::PublishImage() {
    auto image_message = std::make_shared<sensor_msgs::msg::Image>();

    // Access the camera RGBA 8-bit buffer.
    chrono::sensor::UserRGBA8BufferPtr rgba8_ptr =
        simulation_->GetCameraSensor()
            ->GetSensor()
            ->GetMostRecentBuffer<chrono::sensor::UserRGBA8BufferPtr>();

    image_message->header.frame_id = "camera_link";
    image_message->height = rgba8_ptr->Height;
    image_message->width = rgba8_ptr->Width;
    image_message->encoding = "rgba8";
    image_message->step = rgba8_ptr->Width * 4;
    image_message->is_bigendian = false;

    uint img_buffer_step = 4;
    uint img_buffer_size = rgba8_ptr->Height * rgba8_ptr->Width;
    std::vector<uint8_t> image_data(img_buffer_step * img_buffer_size);

    if (rgba8_ptr->Buffer) {
        for (size_t i = 0; i < img_buffer_size; ++i) {
            std::memcpy(&image_data[(img_buffer_step * img_buffer_size) - 1 -
                                    (img_buffer_step * i)],
                        &rgba8_ptr->Buffer[i].A, sizeof(uint8_t));
            // In the following commands I am using a funny notation to
            // write the offset by 4 bytes and 8 bytes respectively.
            std::memcpy(&image_data[(img_buffer_step * img_buffer_size) - 1 -
                                    (img_buffer_step * i + 1)],
                        &rgba8_ptr->Buffer[i].B, sizeof(uint8_t));
            std::memcpy(&image_data[(img_buffer_step * img_buffer_size) - 1 -
                                    (img_buffer_step * i + 2)],
                        &rgba8_ptr->Buffer[i].G, sizeof(uint8_t));
            std::memcpy(&image_data[(img_buffer_step * img_buffer_size) - 1 -
                                    (img_buffer_step * i + 3)],
                        &rgba8_ptr->Buffer[i].R, sizeof(uint8_t));
        }

        image_message->data = nlohmann::json::binary_t(image_data);
    }

    image_publisher_->publish(*image_message);
}

void NativeNode::PublishCameraInfo() {
    auto width = simulation_->GetCameraSensor()->GetSensor()->GetWidth();
    auto height = simulation_->GetCameraSensor()->GetSensor()->GetHeight();
    auto horizontal_fov =
        simulation_->GetCameraSensor()->GetSensor()->GetHFOV();

    auto camera_info_message = std::make_shared<sensor_msgs::msg::CameraInfo>();
    camera_info_message->header.frame_id = "camera_link";
    camera_info_message->header.stamp = get_clock()->now();
    camera_info_message->height = height;
    camera_info_message->width = width;
    camera_info_message->distortion_model = "plumb_bob";

    // Compute the camera focal lenses.
    double aspect_ratio = width / height;
    double vertical_fov =
        2.0 * std::atan(std::tan(horizontal_fov / 2.0) / aspect_ratio);

    // Calculate the projection matrix terms for the focal length
    double focal_length_x = width / tan(horizontal_fov / 2.0);
    double focal_length_y = height / tan(vertical_fov / 2.0);
    double camera_center_x = width / 2.0;
    double camera_center_y = height / 2.0;

    // Compute the projection matrix.
    std::vector<double> K{focal_length_x,
                          0.0,
                          camera_center_x,
                          0.0,
                          focal_length_y,
                          camera_center_y,
                          0.0,
                          0.0,
                          1.0};
    std::copy(K.begin(), K.begin() + K.size(), camera_info_message->k.begin());

    std::vector<double> R{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::copy(R.begin(), R.begin() + R.size(), camera_info_message->r.begin());

    std::vector<double> P{focal_length_x,
                          0.0,
                          camera_center_x,
                          0.0,
                          0.0,
                          focal_length_y,
                          camera_center_y,
                          0.0,
                          0.0,
                          0.0,
                          1.0,
                          0.0};
    std::copy(P.begin(), P.begin() + P.size(), camera_info_message->p.begin());

    camera_info_message->binning_x = 0.0;
    camera_info_message->binning_y = 0.0;
    camera_info_message->roi.x_offset = 0;
    camera_info_message->roi.y_offset = 0;
    camera_info_message->roi.height = 0;
    camera_info_message->roi.do_rectify = false;

    camera_info_publisher_->publish(*camera_info_message);
}

void NativeNode::AckermannCallback(
    ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr
        ackermann_message) {
    simulation_->SetControl(ackermann_message->drive.acceleration,
                            ackermann_message->drive.steering_angle_velocity);
}

bool NativeNode::CheckElapsedTime(const rclcpp::Time& current_time,
                                  rclcpp::Time& last_time,
                                  const double& period) {
    bool is_action_due = (current_time - last_time).seconds() >= period;
    if (is_action_due)
        last_time = current_time;
    return is_action_due;
}

}  // namespace Native
}  // namespace ROS
}  // namespace DYNO
