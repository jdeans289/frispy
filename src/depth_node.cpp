#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <frispy/TFBroadcastPR.h>
#include <frispy/FoundObject.h>
#include <visualization_msgs/Marker.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main (int argc, char** argv) {
	ros::init(argc, argv, "depth");
	ros::NodeHandle node;
    ros::Rate r(1);

    // we will publish on the detected_object topic
    ros::Publisher object_pub = node.advertise<frispy::object>("detected_object", 100);
    FoundObject detected_object(0, 0, 0, object_pub);

    ros::Subscriber boxes_subscriber = node.subscribe("/darknet_ros/bounding_boxes", 100, &FoundObject::processBoxes, &detected_object);

    while (ros::ok()) {
        ros::spinOnce(); 
        r.sleep();
    }
	
  	return 0;
}