/*								kb_input_handler							//
	ros node for handling user input from keyboard; input will be restricted
	to a set of predefined commands (input body twist, adjust gain, potentially
	a set of menu options, etc.

	this file will write to SharedValues::gain, and will publish input twist
	commands to the "input_cmd" topic (to be consumed by the controller node).

	why not use teleop_twist_keyboard? because i hate myself that's why

auth: @dchayto
*/

#include <functional>	// for std::bind
#include <termios.h>	// POSIX terminal control definitions 
#include <iostream>
#include <cstring> 		// for strerror
#include <cstdint>		// for uint8_t (redundant include; already in input)
#include <errno.h>		// error msgs
#include <fcntl.h>		// file controls
#include <unistd.h>		// for STDIN_FILENO
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "input.hpp"

uint8_t commands::gain { 50 };	// from input.hpp 

class KeyboardHandlerNode : public rclcpp::Node
{
public:
	KeyboardHandlerNode()
	 : Node("kb_handler_node")
	{
		// get starting termios settings
		if (tcgetattr(STDIN_FILENO, &ogTermios) != 0)	{
			RCLCPP_ERROR_STREAM(this->get_logger(),
				"Error " << errno << ": " << std::strerror(errno));
		}
		
		configureRaw(ogTermios);	// PBV; don't want to change ogTermios
		
		// PUBLISHERS	
		using namespace std::chrono_literals;
		input_twist_publisher 
		    = this->create_publisher<geometry_msgs::msg::Twist>("input_cmd", 1);
		inputTimer = this->create_wall_timer(500ms,
			std::bind(&KeyboardHandlerNode::onKBTimer, this));
	} // </constructor>

	~KeyboardHandlerNode()	{
		RCLCPP_INFO(this->get_logger(), "KeyboardHandlerNode shutting down");
		tcsetattr(STDIN_FILENO, TCSAFLUSH, &ogTermios); // restore termios settings
	} // </destructor>

private:
	// member variables
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr input_twist_publisher;
	rclcpp::TimerBase::SharedPtr inputTimer;
	struct termios ogTermios; // termios struct before starting program
	
	// member functions
	void onKBTimer();
	void configureRaw(struct termios rawTerm);
}; // </KeyboardHandlerNode>


void KeyboardHandlerNode::onKBTimer()
{
	auto kbTwist = geometry_msgs::msg::Twist(); 

	// check what current keyboard input is
	// translate kb input into command
	static char ch;
	read(STDIN_FILENO, &ch, 1);

	// duplicate assignments, but clearer this way
	kbTwist.linear.x = 0;
	kbTwist.linear.y = 0;
	kbTwist.angular.z = 0;

	// would be nice to be able to handle compound inputs... should look into this
	using namespace KeyboardConstants;
	switch (ch)	{
		// basic movements:
		case FWD:	 	kbTwist.linear.x =  1; 	break;
		case LFT: 		kbTwist.linear.y = -1; 	break;
		case REV: 		kbTwist.linear.x = -1; 	break;
		case RHT:		kbTwist.linear.y =  1; 	break;
		case FWD_RHT: 	kbTwist.linear.x =  1; kbTwist.linear.y =  1; 	break;
		case FWD_LFT:	kbTwist.linear.x =  1; kbTwist.linear.y = -1; 	break;
		case REV_LFT: 	kbTwist.linear.x = -1; kbTwist.linear.y = -1;	break;
		case REV_RHT:	kbTwist.linear.x = -1; kbTwist.linear.y =  1;	break;
		case TRN_LFT: 	kbTwist.angular.z = -1;	break;
		case TRN_RHT: 	kbTwist.angular.z =  1;	break;
		case GAINUP:	
			commands::gain += 1;
			if (commands::gain > 100) commands::gain = 100;
			break;
		case GAINUP10:	// increase gain by 10
			commands::gain += 10;
			if (commands::gain > 100) commands::gain = 100;
			break;
		case GAINDN:	// decrease gain by 1
			if (commands::gain <= 1) { commands::gain = 0; }
			else { commands::gain += 10; }
			break;
		case GAINDN10:	// decrease gain by 10
			if (commands::gain <= 10) { commands::gain = 0; }
			else { commands::gain -= 10; }
			break;
		case 'x':	// VEHICLE STOP
		default: break; 	// invalid command; send {0} twist cmd
	}

	// kbTwist.linear.z = 0;  // don't care about value
	// kbTwist.angular.x = 0;  // don't care about value
	// kbTwist.angular.y = 0;  // don't care about value

	this->input_twist_publisher->publish(kbTwist);	 // publish command
} // </onKBTimer>

void KeyboardHandlerNode::configureRaw(struct termios rawtty)	{
	// copied in by value, so not worried about changing
	cfmakeraw(&rawtty);

	rawtty.c_cflag &= ~CSTOPB;	// clear second stop bit

	rawtty.c_iflag &= ~(IXOFF | IXANY);	// turn off s/w flow control
	
	rawtty.c_cc[VTIME] = 0; // turn off any buffering + waiting for input 
	rawtty.c_cc[VMIN]  = 0; // should return '0' on no input

	tcsetattr(STDIN_FILENO, TCSANOW, &rawtty);
} // </configureRaw>

int main(int argc, char** argv)	{
	rclcpp::init(argc, argv);
	auto kbNode = std::make_shared<KeyboardHandlerNode>();
	rclcpp::spin(kbNode);
	rclcpp::shutdown();

	return 0;
}
