#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>



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

	ros::Subscriber depth_subscriber = node.subscribe("/nav_kinect/depth_registered/image_raw"
													, 100, getDepth);//boost::bind(getDepth, _1, argv[0], argv[1]));
	for (int i = 0; i < 10; i++) {
		ROS_INFO("%d", i);
	}
	ros::spin();
  	return 0;
}