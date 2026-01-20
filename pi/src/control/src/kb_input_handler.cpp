/*								kb_input_handler							//
	ros node for handling user input from keyboard; input will be restricted
	to a set of predefined commands (input body twist, adjust gain, potentially
	a set of menu options, etc.

	this file will write to SharedValues::gain, and will publish input twist
	commands to the "input_cmd" topic (to be consumed by the controller node).

	auth: @dchayto
*/
#include <functional>
#include <ncurses.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/Twist"

#include "input.hpp"

class KeyboardHandlerNode() : public rclcpp::Node
{
public:
	KeyboardHandlerNode()
	 : Node("kb_handler_node")
	{
		// PUBLISHERS	
		input_twist_publisher 
		    = this->create_publisher<geometry_msgs::msg::Twist>("input_cmd", 1);
		inputTimer = this->create_wall_timer(0.5s,
			std::bind(&KeyboardHandlerNode::onKBTimer, this));
	} // </constructor>


private:
	// member variables
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr input_twist_publisher;
	rclcpp::TimerBase::SharedPtr inputTimer;
	
	// member functions
	void onKBTimer()
	{
		// check what current keyboard input is

		// translate kb input into command	

		// publish command

	} // </onKBTimer>
}; // </KeyboardHandlerNode>


int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto kbNode = std::make_shared<KeyboardHandlerNode>();
	rclcpp::spin(kbNode);
	rclcpp::shutdown();

	return 0;
}
