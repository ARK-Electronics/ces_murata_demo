#pragma once

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

class FlightDemo : public rclcpp::Node
{
public:
    FlightDemo();

    void run(); // Main loop for the state machine

private:
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void publishVehicleCommand(int command, float param1 = NAN, float param2 = NAN, float param3 = NAN, float param4 = NAN, 
                               float param5 = NAN, float param6 = NAN, float param7 = NAN);
    rclcpp::TimerBase::SharedPtr _offboard_timer;
    void offboardTimerCallback();
    
    void switchState();
    void arm();
    void takeoff();
    void switchToOffboard();
    void disarm();
    void land();
    void loadParameters();
    

    enum class State
    {
        Idle,
        Arm,
        Takeoff,
        Hover,
        Yaw,
        Land,
        Done
    };

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_pub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_position_sub;

    px4_msgs::msg::VehicleStatus::SharedPtr _vehicle_status;
    px4_msgs::msg::VehicleLocalPosition::SharedPtr _local_position;
    px4_msgs::msg::TrajectorySetpoint _trajectory_setpoint;

    State _state;
    rclcpp::Time _state_start_time;

    bool isStateTimeout(double seconds);

    // Parameters
    float _takeoff_altitude;
    float _hover_duration;
};
