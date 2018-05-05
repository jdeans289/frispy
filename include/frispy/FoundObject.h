#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

class FoundObject {
private:
	int xCenter, yCenter;
	float zDepth;
	darknet_ros_msgs::BoundingBox detected_box;
	geometry_msgs::PointStamped finalLocation;



public:
	FoundObject(const int &xC, const int &yC, const float &zC);

	void getBox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

	void getDepth(const sensor_msgs::ImageConstPtr& msg);

	void getLocation(const sensor_msgs::PointCloud2& msg);

	void buildCube();

	visualization_msgs::Marker marker;
};