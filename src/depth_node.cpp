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



//namespace enc = sensor_msgs::image_encodings;


// given a depth image, extracts the depth of a specified point
// void getDepth(const sensor_msgs::ImageConstPtr& msg) //int x, int y)
// {


//     //int depth = cv_ptr->image.at<short int>(cv::Point(30,30));
//     //ROS_INFO("Depth: %d", depth);
// }

int main (int argc, char** argv) {
	ros::init(argc, argv, "depth");
	ros::NodeHandle node;
    ros::Rate r(1);

    FoundObject detected_object(0, 0, 0);


    ros::Subscriber boxes_subscriber = node.subscribe("/darknet_ros/bounding_boxes", 100, &FoundObject::getBox, &detected_object);
	ros::Subscriber depth_subscriber = node.subscribe("/nav_kinect/depth_registered/hw_registered/image_rect"
													, 100, &FoundObject::getDepth, &detected_object);
                                                    //boost::bind(getDepth, _1, argv[0], argv[1]));
    ros::Subscriber location_subscriber = node.subscribe("/nav_kinect/depth_registered/points"
                                                    , 100, &FoundObject::getLocation, &detected_object);
	
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("detected_object", 1);


    ros::spinOnce();

    while (ros::ok()) {

        marker_pub.publish(detected_object.marker);

        ros::spinOnce();  // calling spinOnce here prevents the cube from being published!

        r.sleep();

    }

    // mapper.receivePose(pose);



 //    for (int i = 0; i < 10; i++) {
	// 	ROS_INFO("%d", i);
	// }
	
  	return 0;
}