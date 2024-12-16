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

    _local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "fmu/out/vehicle_local_position",
    rclcpp::QoS(10).best_effort(),
    std::bind(&FlightDemo::localPositionCallback, this, std::placeholders::_1));

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
    sp.yaw = NAN;          // Yaw in radians
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
                _home_setpoint[0] = _local_position.x;
                _home_setpoint[1] = _local_position.y;
                _home_setpoint[2] = _local_position.z;
                _home_setpoint[3] = _local_position.heading;
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
            _hover_setpoint[2] = _local_position.z - 1.5f; // Hover 1.5 meters above the ground

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
        sp.yaw = _home_setpoint[3]; 
        _trajectory_setpoint_pub->publish(sp);

        if (isStateTimeout(60.0)) // Hover for some time
        {
            RCLCPP_INFO(this->get_logger(), "State: Hover -> Yaw");

            _state = State::Yaw;
            _state_start_time = this->now();
        }

        break;
    }

    case State::Yaw:
    {
        // Step size for yaw increment (1 degree per step)
        const double yaw_step = M_PI / 180.0; // 1 degree in radians
        static double target_yaw = _local_position.heading; // Initialize with current heading
        static double total_yaw_rotated = 0.0; // Total yaw rotated

        // Check if the current heading is close enough to the target yaw
        if (headingReached(target_yaw))
        {
            // Increment the target yaw
            target_yaw += yaw_step;
            total_yaw_rotated += yaw_step;

            // Wrap target_yaw to keep it within -π to π
            if (target_yaw > M_PI) target_yaw -= 2 * M_PI;
            if (target_yaw < -M_PI) target_yaw += 2 * M_PI;

            // Publish the new trajectory setpoint
            px4_msgs::msg::TrajectorySetpoint sp;
            sp.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects microseconds
            sp.position[0] = _hover_setpoint[0];
            sp.position[1] = _hover_setpoint[1];
            sp.position[2] = _hover_setpoint[2];
            sp.yaw = target_yaw; // Update the yaw target
            sp.yawspeed = NAN; // Let PX4 handle the speed
            _trajectory_setpoint_pub->publish(sp);
        }

        // Check if the total yaw rotation is complete (360 degrees)
        if (total_yaw_rotated >= 2 * M_PI) // Full 360-degree rotation
        {
            RCLCPP_INFO(this->get_logger(), "State: Yaw -> Land");
            _state = State::Land;
            _state_start_time = this->now();
            land();

            // Reset yaw tracking variables for next use
            target_yaw = _local_position.heading; // Reset to current heading
            total_yaw_rotated = 0.0;
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

bool FlightDemo::headingReached(float target_heading) const
{
    const double degrees = 1.0;
    const double tolerance = degrees * M_PI / 180.0; // Convert degrees to radians
    // Get the current heading
    auto current_heading = _local_position.heading;

    // Calculate the angular difference
    float delta_heading = target_heading - current_heading;

    // Normalize the angular difference to the range [-π, π]
    while (delta_heading > M_PI) delta_heading -= 2 * M_PI;
    while (delta_heading < -M_PI) delta_heading += 2 * M_PI;

    // Check if the heading difference is within tolerance
    return fabs(delta_heading) < tolerance;
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
