#include <dynamic_tutorials/joint_talker.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_node");
	ros::NodeHandle np;
	
	node_joint::joint_PID node(np);
	node.spin();
//	ros :: spin();
	
	return 0;
}
