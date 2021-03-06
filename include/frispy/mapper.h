#pragma once

#include "frispy/TFBroadcastPR.h"
#include <unordered_map>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <frispy/object.h>
#include <visualization_msgs/Marker.h>

class mapper
{
private:
	ros::Publisher _object_pub;
	ros::Publisher _marker_pub;
	std::unordered_map<std::string, std::vector<geometry_msgs::Pose>> _foundObjects;
	float distanceBetween(geometry_msgs::Point p1, geometry_msgs::Point p2);
	int _marker_color;
	int _object_number;

public:
	mapper(ros::Publisher &opub, ros::Publisher &mpub);

	void storeObject(const frispy::object &object);

	void broadcastAllObjects();

	void broadcastSelectedObjects(std::vector<std::string> &objects);

	void buildMarker(const frispy::object &object);
	
};
