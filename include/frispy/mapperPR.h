#pragma once

#include "frispy/PoseRecipient.h"
#include "frispy/TFBroadcastPR.h"

class mapperPR : public PoseRecipient
{
private:
	TFBroadcastPR _broadcaster;

public:
	mapperPR(TFBroadcastPR& br);

	void recievePose(geometry_msgs::Pose &pose);
	
};