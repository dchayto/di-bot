/*								wheel_odometry.cpp							//
	ros node for handling wheel odometry

	node will take in differential encoder values (delta since last timestep)
	and naively calculate estimated pose and twist of robot

	data will be published on wheel odometry topic, to be consumed by sensor
	fusion node as part of estimating actual robot motion 

	auth: @dchayto
*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


class WheelOdomNode : public rclcpp::Node
{
public:
	WheelOdomNode()
	 : Node("wheel_odom_node")
	{
		// SUBSCRIBERS	
		// node should subscribe to due_hw_interface to receive encoder message


		// PUBLISHERS
		// to start, probably just worry about getting twist publishing since
		// that's what matters for C-L control - pose can wait for plan/map
		odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
		encoderTimer = this->create_wall_timer(1s, 
			[this]()
			{
				auto odomMsg = nav_msgs::msg::Odometry();
				odomMsg.header = ;
				odomMsg.child_frame_id = ;
				odomMsg.pose = ;
				odomMsg.twist = ;
				this->odom_publisher->publish(odomMsg);
			};)


	} // constructor

private:
	// member variables
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;


	// helper functions

}


int main(int argc, char** argv)	{
	rclcpp::init(argc, argv)
	auto wheelOdomNode = std::make_shared<WheelOdomNode>();
	rclcpp::spin(wheelOdomNode);
	rclcpp::shutdown();	
}
