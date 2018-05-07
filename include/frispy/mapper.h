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

public:
	mapper(TFBroadcastPR &br);

	void storeObject(const frispy::object &object);

	// void addObject(geometry_msgs::Pose pose);

	void broadcastAllObjects();

	void broadcastSelectedObjects(std::vector<std::string> &objects);
	
};