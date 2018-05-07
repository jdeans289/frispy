#include <ros/ros.h>
#include "frispy/mapper.h"
#include "frispy/TFBroadcastPR.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper");
	ros::NodeHandle node;
    ros::Rate r(1);
    
    TFBroadcastPR br("odom", "odom");
    mapper m(br);
    ros::Subscriber map_reader = node.subscribe("detected_object", 100, &mapper::storeObject, &m);
    std::vector<std::string> filter{"chair"};

    while (ros::ok()) {

        ros::spinOnce();  

        m.broadcastSelectedObjects(filter);

        r.sleep();

    }
}