#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>



//namespace enc = sensor_msgs::image_encodings;
// void boxGhostCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
//   for (int i = 0; i < msg->bounding_boxes.size(); i++){
//     ROS_INFO("name : %s  xmin: %d  ymin: %d", msg->bounding_boxes[i].Class.c_str(), msg->bounding_boxes[i].xmin, msg->bounding_boxes[i].ymin);
//   }
// }

// int main (int argc, char** argv) {
// 	ros::init(argc, argv, "box_ghost");
// 	ros::NodeHandle node;

	// ros::Subscriber boxes_subscriber = node.subscribe("/darknet_ros/bounding_boxes", 100, boxGhostCallback);
// 	ros::spin();
//   	return 0;
// }