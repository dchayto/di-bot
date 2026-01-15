/*							mech_wheel_controller.hpp						//
	helper functions, variable declarations, structs definitions for mech wheel
	controller

	auth: @dchayto
*/

inline const double WHEEL_RADIUS	{ (60.0 / 2.0) / 1000.0 };		// [m]
inline const double WHEELBASE		{ 0.20 };	// [m]
inline const double TRACK_WIDTH		{ 0.20 };	// [m] - assumes CoM centred

// could really just use an array, but might be nice to be able to refer
// to as "twist.x" instead of twist[0]
struct twist	{
	double x {0.0};		// linear x velocity [m/s]
	double y {0.0};		// linear y velocity [m/s]
	double w {0.0};		// angular z velocity [rad/s]
}


// using v, w output from controller, calculate ith wheelspeed based on 
// vehicle kinematics 
MechWheelControllerNode::getWS(int8_t& wheel_i)	{



}
