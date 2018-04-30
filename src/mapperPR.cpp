#include "frispy/mapperPR.h"

mapperPR::mapperPR(TFBroadcastPR& br) : _broadcaster(br) {}

void mapperPR::receivePose(geometry_msgs::Pose &pose) {
	_broadcaster.receivePose(pose);
}
