#include <dynamic_tutorials/talker.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker_node_2");
	ros::NodeHandle np;
	
	node_example::ExampleTalker node(np);

	ros :: spin();
	
	return 0;
}
