#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>

class FlightDemo : public px4_ros2::ModeBase
{
    public:
        explicit FlightDemo(rclcpp::Node& node);

        // void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

        void onActivate() override;
        void onDeactivate() override;
        void updateSetpoint(float dt_s) override;

    private:

        void loadParameters();

        // Eigen::Vector2f calculateVelocitySetpointXY();
        bool positionReached(const Eigen::Vector3f& target) const;

        enum class State
        {
            Takeoff,
            Idle,
            Waypoint,
            Land,
            Done
        };

        void switchState(State state);
        std::string stateName(State state);

        // ros2
        rclcpp::Node& _node;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

        // px4_ros2_cpp
        std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
        std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
        std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

        State _state = State::Idle;

        bool _land_detected = false;



        // Parameters
        float _param_descent_vel = {};
        float _param_vel_p_gain = {};
        float _param_vel_i_gain = {};
        float _param_max_velocity = {};
        float _param_target_timeout = {};
        float _param_delta_position = {};
        float _param_delta_velocity = {};

        float _vel_x_integral {};
        float _vel_y_integral {};

};