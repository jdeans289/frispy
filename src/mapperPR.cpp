#include "frispy/mapperPR.h"

mapperPR::mapperPR(TFBroadcastPR& br) : _broadcaster(br) {}

void mapperPR::recievePose(geometry_msgs::Pose &pose) {
	_broadcaster.recievePose(pose);
}
