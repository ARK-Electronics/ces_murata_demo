#pragma once

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

class FlightDemo : public rclcpp::Node
{
public:
    FlightDemo();

    void run(); // Main loop for the state machine

private:
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void publishVehicleCommand(int command, float param1 = NAN, float param2 = NAN, float param3 = NAN, float param4 = NAN, 
                               float param5 = NAN, float param6 = NAN, float param7 = NAN);
    void switchState();

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

    px4_msgs::msg::VehicleStatus::SharedPtr _vehicle_status;

    State _state;
    rclcpp::Time _state_start_time;

    bool isStateTimeout(double seconds);
};
