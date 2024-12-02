#include "FlightDemo.hpp"

FlightDemo::FlightDemo()
    : Node("flight_demo"),
      _state(State::Idle)
{
    _vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    _vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", 10, std::bind(&FlightDemo::vehicleStatusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "FlightDemo node initialized.");

    // Run the state machine
    run();
}

void FlightDemo::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    _vehicle_status = msg;
}

void FlightDemo::publishVehicleCommand(int command, float param1, float param2, float param3, float param4,
                                       float param5, float param6, float param7)
{
    px4_msgs::msg::VehicleCommand vehicle_command{};
    vehicle_command.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects microseconds
    vehicle_command.command = command;
    vehicle_command.param1 = param1;
    vehicle_command.param2 = param2;
    vehicle_command.param3 = param3;
    vehicle_command.param4 = param4;
    vehicle_command.param5 = param5;
    vehicle_command.param6 = param6;
    vehicle_command.param7 = param7;
    vehicle_command.target_system = 1;
    vehicle_command.target_component = 1;
    vehicle_command.source_system = 1;
    vehicle_command.source_component = 1;
    vehicle_command.from_external = true;

    _vehicle_command_pub->publish(vehicle_command);
}

void FlightDemo::switchState()
{
    switch (_state)
    {
    case State::Idle:

        if(_vehicle_status->pre_flight_checks_pass)
        {
            RCLCPP_INFO(this->get_logger(), "State: Idle -> Arm");
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
            _state = State::Arm;
            _state_start_time = this->now();
            break;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "State: Idle -> Idle");
            _state = State::Idle;
            _state_start_time = this->now();
            break;
        }

    case State::Arm:
        if (_vehicle_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
        {
            RCLCPP_INFO(this->get_logger(), "State: Arm -> Takeoff");
            _state = State::Takeoff;
            _state_start_time = this->now();
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN, 2.0);
        }
        break;

    case State::Takeoff:
        if (isStateTimeout(5.0)) // Wait for 5 seconds to reach altitude
        {
            RCLCPP_INFO(this->get_logger(), "State: Takeoff -> Hover");
            _state = State::Hover;
            _state_start_time = this->now();
        }
        break;

    case State::Hover:
        if (isStateTimeout(60.0)) // Hover for 5 seconds
        {
            RCLCPP_INFO(this->get_logger(), "State: Hover -> Land");
            _state = State::Land;
            _state_start_time = this->now();
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); // Land
        }
        break;

    case State::Land:
        if (isStateTimeout(5.0)) // Wait for 5 seconds to land
        {
            RCLCPP_INFO(this->get_logger(), "State: Land -> Done");
            _state = State::Done;
        }
        break;

    case State::Done:
        RCLCPP_INFO(this->get_logger(), "Mission complete.");
        break;
    }
}

bool FlightDemo::isStateTimeout(double seconds)
{
    return (this->now() - _state_start_time).seconds() > seconds;
}

void FlightDemo::run()
{
    while (rclcpp::ok() && _state != State::Done)
    {
        switchState();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
