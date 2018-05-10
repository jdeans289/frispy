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
	ros::Publisher _objectPub;
	ros::Publisher _markerPub;
	std::unordered_map<std::string, std::vector<geometry_msgs::Pose>> _foundObjects;
	float distanceBetween(geometry_msgs::Point p1, geometry_msgs::Point p2);
	int _markerColor;
	int _objectNumber;

public:
	mapper(ros::Publisher &opub, ros::Publisher &mpub);

	void storeObject(const frispy::object &object);

	void broadcastAllObjects();

	void broadcastSelectedObjects(std::vector<std::string> &objects);

	void buildMarker(const frispy::object &object);
	
};
