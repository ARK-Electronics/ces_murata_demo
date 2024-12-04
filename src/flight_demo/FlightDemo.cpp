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
    _trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    _offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    // Start the offboard timer
    _offboard_timer = this->create_wall_timer(
        std::chrono::milliseconds(100), // Timer period: 100 ms
        std::bind(&FlightDemo::offboardTimerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "FlightDemo node initialized.");

    // // Run the state machine
    // run();
}

void FlightDemo::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    _vehicle_status = msg;
    // RCLCPP_INFO(this->get_logger(), "Vehicle status received:");
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
    px4_msgs::msg::TrajectorySetpoint trajectory_setpoint{};
    trajectory_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects microseconds
    trajectory_setpoint.position[0] = 0.0;   // Setpoint X position in meters
    trajectory_setpoint.position[1] = 0.0;   // Setpoint Y position in meters
    trajectory_setpoint.position[2] = -2.0;  // Altitude in meters
    trajectory_setpoint.yawspeed = 0.2; // Yaw in radians

    _trajectory_setpoint_pub->publish(trajectory_setpoint);
}
void FlightDemo::arm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // Arm
}

void FlightDemo::takeoff()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN, 2.0);
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
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // Set mode to offboard
}

void FlightDemo::switchState()
{
    switch (_state)
    {
    case State::Idle:

        if(_vehicle_status && _vehicle_status->pre_flight_checks_pass)
        {
            RCLCPP_INFO(this->get_logger(), "State: Idle -> Arm");
            arm();
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
        if (_vehicle_status && _vehicle_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
        {
            RCLCPP_INFO(this->get_logger(), "State: Arm -> Takeoff");
            _state = State::Takeoff;
            _state_start_time = this->now();
            takeoff();
        }
        break;

    case State::Takeoff:
        if (_vehicle_status && _vehicle_status->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) // Wait for 5 seconds to reach altitude
        {   
            switchToOffboard();
            RCLCPP_INFO(this->get_logger(), "State: Takeoff -> Hover");
            _state = State::Hover;
            _state_start_time = this->now();
        }
        break;

    case State::Hover:
        if (isStateTimeout(60.0)) // Hover for 5 seconds
        {
            RCLCPP_INFO(this->get_logger(), "State: Hover -> Yaw");
            _state = State::Yaw;
            _state_start_time = this->now();
            // uint16 VEHICLE_CMD_CONDITION_YAW = 115			# Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_CONDITION_YAW, 360.0, 10.0, 1.0, 1.0);
        }
        break;

    case State::Yaw:
        if (isStateTimeout(60.0)) // Wait for 5 seconds to yaw
        {
            RCLCPP_INFO(this->get_logger(), "State: Yaw -> Land");
            _state = State::Land;
            _state_start_time = this->now();
            land();
        }
        break;

    case State::Land:
        if (isStateTimeout(5.0)) // Wait for 5 seconds to land
        {
            RCLCPP_INFO(this->get_logger(), "State: Land -> Done");
            disarm();
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
        rclcpp::spin_some(this->get_node_base_interface()); // Process incoming messages
        switchState(); // Execute state machine logic
        rclcpp::sleep_for(std::chrono::milliseconds(100)); // Small delay to reduce CPU usage
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightDemo>();
    // rclcpp::spin(node);
    node->run();
    rclcpp::shutdown();
    return 0;
}
