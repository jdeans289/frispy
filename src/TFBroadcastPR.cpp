#include "frispy/TFBroadcastPR.h"

TFBroadcastPR::TFBroadcastPR(std::string s1, std::string s2) : fromFrame(s1), toFrame(s2) {}

void TFBroadcastPR::receivePose(geometry_msgs::Pose &pose) {
	tf::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
	tf::Quaternion orientation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Transform t(orientation, position);
	
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), fromFrame, toFrame));
}