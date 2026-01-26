/*								imu_handler.cpp							//
	ros node for interfacing with onboard imu

	node will take in current imu values 
	and naively calculate estimated change in robot pose

	pose will be published on imu data topic, to be consumed by sensor
	fusion node as part of estimating actual robot pose/motion 

	auth: @dchayto
*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class IMUHandlerNode : public rclcpp::Node
{
public:
	IMUHandlerNode()
	 : Node("imu_handler_node")
	{
		// PUBLISHERS
		// for now, just worry about getting twist publishing since that's what
		// matters for C-L control - pose can wait for mapping/planning stage
		imu_publisher = this->create_publisher<nav_msgs::msg::Odometry>("imu", 10);
		imuTimer = this->create_wall_timer(1s, 
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
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr imu_publisher;


	// helper functions

}


int main(int argc, char** argv)	{
	rclcpp::init(argc, argv)
	auto imuNode = std::make_shared<IMUHandlerNode:>();
	rclcpp::spin(imuNode);
	rclcpp::shutdown();	
}
