#include <dynamic_tutorials/vel_talker.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vel_node");
	ros::NodeHandle np;
	
	node_vel::vel_PID node(np);
	node.spin();
//	ros :: spin();
	
	return 0;
}
