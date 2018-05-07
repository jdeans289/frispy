#include <ros/ros.h>
#include "frispy/mapper.h"
#include "frispy/TFBroadcastPR.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapper");
	ros::NodeHandle node;
    ros::Rate r(1);
    
    //TFBroadcastPR br("odom", "odom");

    ros::Publisher object_pub = node.advertise<frispy::object>("detected_objects", 100);
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualized_objects", 1);
    mapper m(object_pub, marker_pub);
    ros::Subscriber map_reader = node.subscribe("all_detected_objects", 100, &mapper::storeObject, &m);
    

    while (ros::ok()) {

        ros::spinOnce();  
        m.broadcastAllObjects();
        // ROS_INFO("Object class: %s\nObject XYZ: %f %f %f\n", detected_object.thisObject.Class.c_str(), detected_object.thisObject.location.point.x, detected_object.thisObject.location.point.y, detected_object.thisObject.location.point.z);
        // object_pub.publish(detected_object.thisObject);

        r.sleep();

    }
}