#include <ros/ros.h>
#include "frispy/mapper.h"
#include "frispy/TFBroadcastPR.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper");
	ros::NodeHandle node;

    TFBroadcastPR br("odom", "odom");
    mapper m(br);
    ros::Subscriber map_reader = node.subscribe("depth/detected_object", 100, &mapper::storeObject, &m);
}