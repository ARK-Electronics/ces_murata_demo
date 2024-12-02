#include "FlightDemo.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "FlightDemoCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

FlightDemo::FlightDemo(rclcpp::Node& node)
    : ModeBase(node, kModeName),
      _node(node)
{
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
				     rclcpp::QoS(1).best_effort(), std::bind(&FlightDemo::vehicleLandDetectedCallback, this, std::placeholders::_1));

    _vehicle_status_sub = _node.create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status",
                     rclcpp::QoS(1).best_effort(), std::bind(&FlightDemo::vehicleStatusCallback, this, std::placeholders::_1));
    _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 1);

	loadParameters();
    float offset = 0.5;
    float approach = 1.0;
    _target_positions.push_back(Eigen::Vector3f(0.0, 0.0, -approach));
    _target_positions.push_back(Eigen::Vector3f(offset, offset, -approach));
    _target_positions.push_back(Eigen::Vector3f(offset, -offset, -approach));
    _target_positions.push_back(Eigen::Vector3f(-offset, -offset, -approach));
    _target_positions.push_back(Eigen::Vector3f(-offset, offset, -approach));
    _target_positions.push_back(Eigen::Vector3f(0.0, 0.0, -approach));
}

void FlightDemo::loadParameters()
{
	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
}

void FlightDemo::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    _vehicle_status = msg;
}

void FlightDemo::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    _land_detected = msg->landed;
}

// Vehicle command can be used for Arming, Disarming, Takeoff, Land
void FlightDemo::vehicleCommandPub(int command, float param1, float param2 = std::nan(""), float param3= std::nan(""),float param4= std::nan(""), float param5= std::nan(""),float param6= std::nan(""), float param7 = std::nan(""))
{
    px4_msgs::msg::VehicleCommand vehicle_command{};
    vehicle_command.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
    vehicle_command.command = command;  // Command ID
    vehicle_command.param1 = param1;
    vehicle_command.param2 = param2;
    vehicle_command.param3 = param3;
    vehicle_command.param4 = param4;
    vehicle_command.param5 = param5;
    vehicle_command.param6 = param6;
    vehicle_command.param7 = param7;    // Altitude value during takeoff
    vehicle_command.target_system = 1;  // System which should execute the command
    vehicle_command.target_component = 1;   // Component which should execute the command
    vehicle_command.source_system = 1;  // System which sent the command
    vehicle_command.source_component = 1;   // Component which sent the command
    vehicle_command.from_external = true; // Indicate it's from an external source
    _vehicle_command_pub->publish(vehicle_command);
}

void FlightDemo::onActivate()
{
    RCLCPP_INFO(_node.get_logger(), "FlightDemo activated");
    RCLCPP_INFO(_node.get_logger(), "Idle for 1 minute");
    _action_time_started = _node.now();
    _state = State::Idle;
    _land_detected = false;
    
}
void FlightDemo::onDeactivate()
{
    // no op
}

void FlightDemo::updateSetpoint(float dt_s)
{
    switch (_state)
    {
        case State::Idle:{
            
            // Idle for 1 minute
            if (_node.now() - _action_time_started > 60s){
                _action_time_started = _node.now();
                switchState(State::Waypoint);
            }


            break;
        }
        case State::Takeoff:{
            // publish arm command
            // vehicleCommandPub(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            // RCLCPP_INFO(_node.get_logger(), "Arming vehicle");

            // if (_vehicle_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED){
            //     vehicleCommandPub(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 1.0, 0.0, 2.0);
            // }

            if(_vehicle_local_position->positionNed().z() <= -2.0){
                switchState(State::Idle);
            }
            



            break;
        }
        case State::Waypoint:{
            // Go to rectengular waypoints
            Eigen::Vector3f target = _target_positions[_target_index];
            if (positionReached(target)){
                _target_index++;
                RCLCPP_INFO(_node.get_logger(), "Waypoint reached");
                if (_target_index >= _target_positions.size()){
                    RCLCPP_INFO(_node.get_logger(), "All waypoints reached, starting yaw");
                    _action_time_started = _node.now();
                    switchState(State::Yaw);
                }
            }
            else{
                _trajectory_setpoint->updatePosition(target);
            }

            break;
        }
        case State::Yaw:{
            // When I do this, the drone will yaw around for 360 degrees slowly, but also shift in the x and y direction and goes up and down
            // Yaw around for 360 degrees slowly
            _trajectory_setpoint->update(
                Eigen::Vector3f(NAN, NAN, NAN),
                std::nullopt,
                std::nullopt,
                _yaw_speed
            );
            
            if((_node.now() - _action_time_started > 60s)&&(_logic_flag == false)){
                _yaw_speed = -_yaw_speed;
                _logic_flag = true;
            }

            if(_node.now() - _action_time_started > 120s){
                _trajectory_setpoint->update(
                Eigen::Vector3f(0.0, 0.0, 0.0),
                std::nullopt,
                std::nullopt,
                std::nullopt
            );
                switchState(State::Land);
               
            }
            // vehicleCommandPub(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_CONDITION_YAW, 1.0, 0.0, 2.0);

            

            break;
        }
        case State::Land:{
            _trajectory_setpoint->update(
                Eigen::Vector3f(0.0, 0.0, _param_descent_vel),
                std::nullopt,
                std::nullopt,
                std::nullopt
            );

        if (_land_detected) {
			switchState(State::Done);
		}
            break;
        }
        case State::Done:
            break;
        default:
            break;
    }
}

// Eigen::Vector2f FlightDemo::calculateVelocitySetpointXY()
// {
// 	float p_gain = _param_vel_p_gain;
// 	float i_gain = _param_vel_i_gain;

// 	// P component
// 	float delta_pos_x = _vehicle_local_position->positionNed().x() // - _tag.position.x();
// 	float delta_pos_y = _vehicle_local_position->positionNed().y() // - _tag.position.y();

// 	// I component
// 	_vel_x_integral += delta_pos_x;
// 	_vel_y_integral += delta_pos_y;
// 	float max_integral = _param_max_velocity;
// 	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
// 	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

// 	float Xp = delta_pos_x * p_gain;
// 	float Xi = _vel_x_integral * i_gain;
// 	float Yp = delta_pos_y * p_gain;
// 	float Yi = _vel_y_integral * i_gain;

// 	// Sum P and I gains
// 	float vx = -1.f * (Xp + Xi);
// 	float vy = -1.f * (Yp + Yi);

// 	// 0.1m/s min vel and 3m/s max vel
// 	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
// 	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

// 	return Eigen::Vector2f(vx, vy);
// }

bool FlightDemo::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

std::string FlightDemo::stateName(State state)
{
    switch (state)
    {
        case State::Takeoff:
            return "Takeoff";
        case State::Idle:
            return "Idle";
        case State::Waypoint:
            return "Waypoint";
        case State::Yaw:
            return "Yaw";
        case State::Land:
            return "Land";
        case State::Done:
            return "Done";
        case State::Interrupt:
            return "Interrupt";
        default:
		return "Unknown";
    }
}

void FlightDemo::switchState(State state)
{
    RCLCPP_INFO(_node.get_logger(), "Switching state from %s to %s", stateName(_state).c_str(), stateName(state).c_str());
    _state = state;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<FlightDemo>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}