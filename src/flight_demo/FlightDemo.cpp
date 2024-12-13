#include "FlightDemo.hpp"

FlightDemo::FlightDemo()
    : Node("flight_demo"),
      _state(State::Idle)
{
    _vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    _vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status",
    rclcpp::QoS(1).best_effort(), // Use SensorDataQoS for PX4 compatibility
    std::bind(&FlightDemo::vehicleStatusCallback, this, std::placeholders::_1));

    _local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("fmu/out/local_position", 10, std::bind(&FlightDemo::localPositionCallback, this, std::placeholders::_1));
    _trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    _offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    _state_start_time = this->now();

    // Start the offboard timer
    _offboard_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), // Timer period: 100 ms
        std::bind(&FlightDemo::offboardTimerCallback, this)
    );

    px4_msgs::msg::TrajectorySetpoint sp;
    sp.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects microseconds
    sp.position[0] = NAN;   // Setpoint X position in meters
    sp.position[1] = NAN;   // Setpoint Y position in meters
    sp.position[2] = NAN;  // Altitude in meters
    _trajectory_setpoint_pub->publish(sp);

    RCLCPP_INFO(this->get_logger(), "FlightDemo node initialized.");
}

void FlightDemo::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    _vehicle_status = *msg;
}

void FlightDemo::localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    _local_position = *msg;
}

void FlightDemo::loadParameters()
{
    this->declare_parameter<float>("takeoff_altitude", 1.5);
    this->declare_parameter<float>("hover_duration", 60.0);

    this->get_parameter("takeoff_altitude", _takeoff_altitude);
    this->get_parameter("hover_duration", _hover_duration);
}

void FlightDemo::runStateMachine()
{
    switch (_state)
    {
    case State::Idle:
    {
        if (_vehicle_status.pre_flight_checks_pass)
        {
            // Switch to offboard after 2 seconds
            if (isStateTimeout(2.0)) {
                switchToOffboard();
                _state = State::Arm;
                _state_start_time = this->now();
                RCLCPP_INFO(this->get_logger(), "State: Idle -> Arm");
            }
        }

        break;
    }

    case State::Arm:
    {
        if (_vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
            arm();
            _state = State::Takeoff;
            _state_start_time = this->now();
            RCLCPP_INFO(this->get_logger(), "State: Arm -> Takeoff");
        }

        break;
    }

    case State::Takeoff:
    {

        if (_vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
        {
            _hover_setpoint[0] = _local_position.x;
            _hover_setpoint[1] = _local_position.y;
            _hover_setpoint[2] = _local_position.z - 2.f;

            _state = State::Hover;
            _state_start_time = this->now();
            RCLCPP_INFO(this->get_logger(), "State: Takeoff -> Hover");
        }

        break;
    }

    case State::Hover:
    {
        px4_msgs::msg::TrajectorySetpoint sp;
        sp.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects microseconds
        sp.position[0] = _hover_setpoint[0];
        sp.position[1] = _hover_setpoint[1];
        sp.position[2] = _hover_setpoint[2];
        _trajectory_setpoint_pub->publish(sp);

        if (isStateTimeout(60.0)) // Hover for some time
        {
            RCLCPP_INFO(this->get_logger(), "State: Hover -> Land");

            _state = State::Land;
            _state_start_time = this->now();
        }

        break;
    }

    case State::Yaw:
    {
        // _trajectory_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects microseconds
        // _trajectory_setpoint.yaw = 3.14; // Yaw 90 degrees
        if (isStateTimeout(60.0)) // Wait for 5 seconds to yaw
        {
            RCLCPP_INFO(this->get_logger(), "State: Yaw -> Land");
            _state = State::Land;
            _state_start_time = this->now();
            land();
        }

        break;
    }

    case State::Land:
    {
        if (isStateTimeout(5.0)) // Wait for 5 seconds to land
        {
            RCLCPP_INFO(this->get_logger(), "State: Land -> Done");
            disarm();
            _state = State::Done;
        }

        break;
    }

    case State::Done:
    {
        RCLCPP_INFO(this->get_logger(), "Mission complete.");
        break;
    }
    } // end switch-case
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

void FlightDemo::offboardTimerCallback()
{
    px4_msgs::msg::OffboardControlMode offboard_control_mode{};
    offboard_control_mode.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects microseconds
    offboard_control_mode.position = true;
    offboard_control_mode.velocity = false;
    offboard_control_mode.acceleration = false;
    offboard_control_mode.attitude = false;
    offboard_control_mode.body_rate = false;
    _offboard_control_mode_pub->publish(offboard_control_mode);
}

void FlightDemo::arm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
}

void FlightDemo::disarm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); // Disarm
}

void FlightDemo::land()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); // Land
}

void FlightDemo::switchToOffboard()
{
    RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode");
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // Set mode to offboard
}

bool FlightDemo::isStateTimeout(double seconds)
{
    return (this->now() - _state_start_time).seconds() > seconds;
}

void FlightDemo::run()
{
    while (rclcpp::ok() && _state != State::Done)
    {
        rclcpp::spin_some(this->get_node_base_interface()); // Process incoming messages
        runStateMachine();
        rclcpp::sleep_for(std::chrono::milliseconds(10)); // Small delay to reduce CPU usage
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightDemo>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
