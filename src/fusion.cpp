#include <ros/ros.h>
#include "RangeFusion.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DistanceFusion");
	ros::NodeHandle node;
	RangeFusion range_fusion_node(node, 20, 0.1f, 40.0f);

	range_fusion_node.run();

	return 0;
}
