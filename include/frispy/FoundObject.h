#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <math.h>

class FoundObject {
private:
	int xCenter, yCenter;
	float zCenter;
	darknet_ros_msgs::BoundingBox detected_box;


public:
	FoundObject(const int &xC, const int &yC, const float &zC);

	void getDepth(const sensor_msgs::ImageConstPtr& msg);

	void getBox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

	void buildCube();

	visualization_msgs::Marker marker;
};