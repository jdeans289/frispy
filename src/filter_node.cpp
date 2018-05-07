#include <ros/ros.h>
#include "frispy/mapper.h"
#include "frispy/TFBroadcastPR.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper");
	ros::NodeHandle node;
    ros::Rate r(1);
    
<<<<<<< HEAD
    TFBroadcastPR br("odom", "odom");
    mapper m(br);
    ros::Subscriber map_reader = node.subscribe("detected_object", 100, &mapper::storeObject, &m);
    std::vector<std::string> filter{"chair"};
=======
    //TFBroadcastPR br("odom", "odom");

    ros::Publisher object_pub = node.advertise<frispy::object>("detected_objects", 100);
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualized_objects", 1);
    mapper m(object_pub, marker_pub);
    ros::Subscriber map_reader = node.subscribe("all_detected_objects", 100, &mapper::storeObject, &m);
    
>>>>>>> e9ea5b27bb23450535d8c06414434d5cd3e5a0d9

    while (ros::ok()) {

        ros::spinOnce();  

        m.broadcastSelectedObjects(filter);

        r.sleep();

    }
}