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

	loadParameters();
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

void FlightDemo::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    _land_detected = msg->landed;
}

// void FlightDemo::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     RCLCPP_INFO(_node.get_logger(), "TODO: Implement targetPoseCallback");
// }

void FlightDemo::onActivate()
{
    _state = State::Takeoff;
    _land_detected = false;
}
void FlightDemo::onDeactivate()
{
    _state = State::Idle;
}

void FlightDemo::updateSetpoint(float dt_s)
{
    switch (_state)
    {
        case State::Idle:{
            break;
        }
        case State::Takeoff:{
            break;
        }
        case State::Waypoint:{
            break;
        }
        case State::Land:{
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
        case State::Land:
            return "Land";
        case State::Done:
            return "Done";
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