#pragma once

#include "frispy/TFBroadcastPR.h"
#include <unordered_map>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <frispy/object.h>

class mapper
{
private:
	TFBroadcastPR _broadcaster;
	std::unordered_map<std::string, std::vector<geometry_msgs::Pose>> _foundObjects;
	float distanceBetween(geometry_msgs::Point p1, geometry_msgs::Point p2);

public:
	mapper(TFBroadcastPR &br);

	void storeObject(const frispy::object &object);

	void broadcastAllObjects();

	void broadcastSelectedObjects(std::vector<std::string> &objects);
	
};