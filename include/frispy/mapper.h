#pragma once

#include "frispy/TFBroadcastPR.h"
#include <unordered_map>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>

class mapper
{
private:
	TFBroadcastPR _broadcaster;
	std::unordered_map<std::string, std::vector<geometry_msgs::Pose>> _foundObjects;

public:
	mapper(TFBroadcastPR &br);

	void receivePose(const geometry_msgs::Pose &pose);

	// void addObject(geometry_msgs::Pose pose);

	void broadcastAllObjects();

	void broadcastSelectedObjects(std::vector<std::string> &objects);
	
};