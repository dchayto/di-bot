/*							mech_wheel_controller.cpp				     	//
	ros node for robot controller 

	node will take in a body twist vector representing an input command, as 
	well as a secondary twist vector representing the current actual robot 
	twist, as determined from onboard sensors and wheel odometry

	robot will then convert these values into wheelspeed commands, and send
	to due's hardware interface node 

	auth: @dchayto
*/

#include <cstdint>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control/msg/wheelspeed.hpp"

#include "mec_wheel_controller.hpp" 	// splitting up helper functions/consts
#include "input.hpp"

class MechWheelControllerNode : public rclcpp::Node
{
public:
	MechWheelControllerNode()
	 : Node("mech_controller_node")
	{
		// SUBSCRIBERS
		input_twist_subscription = this->create_subscription<geometry_msgs::msg::Twist>
			("input_cmd", 1,
			[this](const geometry_msgs::msg::Twist& icMsg)
			{
				cmdTwist.x = icMsg.linear.x;
				cmdTwist.y = icMsg.linear.y;
				cmdTwist.w = icMsg.angular.z;
			});
		measured_twist_subscription = this->create_subscription<geometry_msgs::msg::Twist>
			("bel_twist", 1,
			[this](const geometry_msgs::msg::Twist& btMsg)
			{
				belTwist.x = btMsg.linear.x;
				belTwist.y = btMsg.linear.y;
				belTwist.w = btMsg.angular.z;
			});
	

		// PUBLISHERS
		using namespace std::chrono_literals;
		ws_publisher = this->create_publisher<control::msg::Wheelspeed>("wheelspeed", 1);
		wsTimer = this->create_wall_timer(1s,
			[this]()
			{
				// get control signal; for now, just considering direction of
				// velocity vectors, not magnitude

				// pre-computing scale factor; equivalent to norm both vectors,
				// then multiply by magnitude of cmd vector
				static double twistSF;
				twistSF = cmdTwist.getLength() / belTwist.getLength(); 

				ctrlTwist.x = cmdTwist.x - twistSF*belTwist.x;
				ctrlTwist.y = cmdTwist.y - twistSF*belTwist.y;
				ctrlTwist.w = cmdTwist.w - twistSF*belTwist.w;

				// for now, scaling to [-127 127] based on set gain
				static double fr, fl, br, bl;

				fr = this->getFrontRightWS();
				fl = this->getFrontLeftWS();
				br = this->getBackRightWS();
				bl = this->getBackLeftWS();

				// this sucks
				static double wheelSF;
				wheelSF = (SharedValues::gain * 127) / std::max({
					std::abs(fr), std::abs(fl), std::abs(br), std::abs(bl)});

				// using control signal, get wheelspeeds
				auto wsMsg = control::msg::Wheelspeed();
				wsMsg.front_right 	= static_cast<int8_t>(fr * wheelSF);
				wsMsg.front_left 	= static_cast<int8_t>(fl * wheelSF); 
				wsMsg.back_right 	= static_cast<int8_t>(br * wheelSF); 
				wsMsg.back_left 	= static_cast<int8_t>(bl * wheelSF); 
				this->ws_publisher->publish(wsMsg);
			});
		
	} // </constructor>

	~MechWheelControllerNode()
	{
		RCLCPP_INFO(this->get_logger(), "MechWheelControllerNode shutting down.");
	} // </destructor>

private:
	// member variables
	// note: might not be practicable to use velocities directly; should
	// consider just using relative values for body twists, and converting
	// sensor twist value into relative values
	twist cmdTwist {0.0, 0.0, 0.0}; // commanded twist [x, y, w]
	twist belTwist {0.0, 0.0, 0.0}; // belief twist [x, y, w]
	twist ctrlTwist {0.0, 0.0, 0.0};	// control signal twist [x, y, w]

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_twist_subscription;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr measured_twist_subscription;
	rclcpp::Publisher<control::msg::Wheelspeed>::SharedPtr ws_publisher;
	rclcpp::TimerBase::SharedPtr wsTimer;

	// helper functions
	int8_t getFrontRightWS();
	int8_t getFrontLeftWS();
	int8_t getBackRightWS();
	int8_t getBackLeftWS();

}; // class


int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto controllerNode = std::make_shared<MechWheelControllerNode>();
	rclcpp::spin(controllerNode);
	rclcpp::shutdown();

	return 0;
} // main
