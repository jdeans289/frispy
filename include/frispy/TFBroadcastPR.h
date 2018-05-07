#pragma once

#include "frispy/PoseRecipient.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

class TFBroadcastPR : public PoseRecipient {
private:
	tf::TransformBroadcaster br;
	const std::string& fromFrame;
	const std::string& toFrame;

public:
	TFBroadcastPR(std::string s1, std::string s2);
	void receivePose(geometry_msgs::Pose &pose);
};
