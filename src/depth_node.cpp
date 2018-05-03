#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <frispy/TFBroadcastPR.h>
#include <frispy/mapperPR.h>
#include <visualization_msgs/Marker.h>



//namespace enc = sensor_msgs::image_encodings;

// given a depth image, extracts the depth of a specified point
void getDepth(const sensor_msgs::ImageConstPtr& msg) //int x, int y)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg); //enc::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ROS_INFO("Rows: %d", cv_ptr->image.rows);
    ROS_INFO("Cols: %d", cv_ptr->image.cols);
    int rows = cv_ptr->image.rows;
    int cols = cv_ptr->image.cols;

    for (int r = 0; r < rows; r++) {
    	for (int c = 0; c < 531; c++) {
    		ROS_INFO("Depth at %d,%d: %d", r, c, cv_ptr->image.at<int>(cv::Point(r,c)));
    	}
    }

    //int depth = cv_ptr->image.at<short int>(cv::Point(30,30));
    //ROS_INFO("Depth: %d", depth);
}

int main (int argc, char** argv) {
	ros::init(argc, argv, "depth");
	ros::NodeHandle node;
    ros::Rate r(1);

	ros::Subscriber depth_subscriber = node.subscribe("/nav_kinect/depth_registered/image_raw"
													, 100, getDepth);//boost::bind(getDepth, _1, argv[0], argv[1]));
	
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("detected_object", 1);
    uint32_t shape = visualization_msgs::Marker::CUBE;

    


    while (ros::ok()) {

        visualization_msgs::Marker marker;

        marker.header.frame_id = "nav_kinect_rgb_optical_frame";
        marker.header.stamp = ros::Time::now();
      
         // Set the namespace and id for this marker.  This serves to create a unique ID
             // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;


        // TFBroadcastPR broadcaster("odom","camera_rgb_optical_frame");
        // mapperPR mapper(broadcaster);

        geometry_msgs::Pose pose;
        pose.position.x = 1;
        pose.position.y = 1;
        pose.position.z = 1;

        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;


        // set scale
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // set color
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        
        marker.pose = pose;
        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);

        //ros::spinOnce();  // calling spinOnce here prevents the cube from being published!

        r.sleep();

    }

    // mapper.receivePose(pose);



 //    for (int i = 0; i < 10; i++) {
	// 	ROS_INFO("%d", i);
	// }
	
  	return 0;
}