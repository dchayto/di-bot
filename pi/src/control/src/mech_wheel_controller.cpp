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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/Twist"
#include "control/msg/wheelspeed.hpp"

#include "mech_wheel_controller.hpp" 	// splitting up helper functions

class MechWheelControllerNode : public rclcpp::Node
{
public:
	MechWheelControllerNode()
	 : Node("mech_controller_node")
	{
		// SUBSCRIBERS
		// node should subscribe to controller input message, as well as
		// body twist message from sensor fusion; these should be fed into
		// the controller to set u and v, which are then fed to the kinematic
		// model to determine the required wheelspeed commands

		
		// PUBLISHERS
		// node should 

	} // constructor





private:
	// member variables
	// note: might not be practicable to use velocities directly; should
	// consider just using relative values for body twists, and converting
	// sensor twist value into relative values
	rclcpp::Publisher<control::msg::Wheelspeed>::SharedPtr ws_publisher;
	twist twist_cmd {0.0, 0.0, 0.0}; // commanded twist [x, y, w]
	twist twist_msr {0.0, 0.0, 0.0}; // measured twist [x, y, w]

	// helper functions
	int8_t getWS(int8_t& wheel_i);		// all other params are member vars

}; // class


int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto controllerNode = std::make_shared<MechWheelControllerNode>();
	rclcpp::spin(controllerNode);
	rclcpp::shutdown();

	return 0;
} // main
