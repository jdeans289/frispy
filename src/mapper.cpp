#include "frispy/mapper.h"

mapper::mapper(ros::Publisher &opub, ros::Publisher &mpub) : _object_pub(opub), _marker_pub(mpub), _foundObjects(), _marker_color(1) {}

float mapper::distanceBetween(geometry_msgs::Point p1, geometry_msgs::Point p2) {
	auto diffX = p2.x - p1.x;
	auto diffY = p2.y - p1.y;
	auto diffZ = p2.z - p1.z;
	return std::sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
}
//Recieves pose and object class string
void mapper::storeObject(const frispy::object &object) {
	std::vector<geometry_msgs::Pose> *previouslyFound;
	
	if(_foundObjects.find(object.Class) != _foundObjects.end()) {
		previouslyFound = &_foundObjects.find(object.Class)->second;
		ROS_INFO("Already have %s", object.Class.c_str());		
		for(auto pose : *previouslyFound) {	
			if(distanceBetween(pose.position, object.location.pose.position) < .1) {
				return;
			}
		}
		previouslyFound->push_back(object.location.pose);
	}
	else {
		std::vector<geometry_msgs::Pose> newList;
		newList.push_back(object.location.pose);
		_foundObjects.emplace(std::make_pair(object.Class, newList));

	}
	ROS_INFO("Stored %s", object.Class.c_str());
}

void mapper::broadcastAllObjects() {
	for(auto& object : _foundObjects) {
		//broadcasts the pose of every object in _foundObjects
		for(auto &location : object.second) {
			frispy::object thisObject;
			thisObject.location.pose = location;
			thisObject.Class = object.first;
			_object_pub.publish(thisObject);
			buildMarker(thisObject);
			//_broadcaster.receivePose(location);
		}
	}
}

void mapper::broadcastSelectedObjects(std::vector<std::string> &objects) {
	for(auto &objectType : objects) {
		auto match = _foundObjects.find(objectType);
		if(match != _foundObjects.end()) {
			for(auto &location : match->second) {
				frispy::object thisObject;
				thisObject.location.pose = location;
				thisObject.Class = match->first;
				_object_pub.publish(thisObject);
				buildMarker(thisObject);
			}
		}
		else {
			ROS_INFO("No %ss have been found.", objectType);
		}
	}
}

void mapper::buildMarker(const frispy::object &thisObject) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = thisObject.Class;
	//marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;


  	// set scale
  	marker.scale.x = 0.1;
  	marker.scale.y = 0.1;
  	marker.scale.z = 0.1;

 	// set color
 	marker.color.r = 0.0f;
 	marker.color.g = 0.0f;
 	marker.color.b = 0.0f;
 	switch (_marker_color % 3) {
 		case 0: marker.color.r = 1.0f;
 				break;
 		case 1: marker.color.g = 1.0f;
 				break;
 		case 2: marker.color.b = 1.0f;
 				break;
 	}
 	_marker_color++;

 	marker.color.a = 1.0;

	// TFBroadcastPR broadcaster("odom","camera_rgb_optical_frame");
	// mapperPR mapper(broadcaster);

 	marker.pose = thisObject.location.pose;
 	marker.lifetime = ros::Duration();
 	_marker_pub.publish(marker);

	//geometry_msgs::Pose finalPose;

 //  ROS_INFO("finalLocation XYZ: %lf, %lf, %lf", finalLocation.point.x, finalLocation.point.y, finalLocation.point.z);


 //  // put the calculated point into the pose
 //  finalPose.position = finalLocation.point;

	// finalPose.orientation.x = 0;
	// finalPose.orientation.y = 0;
	// finalPose.orientation.z = 0;
	// finalPose.orientation.w = 1;
  

}

/*
//Recieves pose and object class string
void mapper::receivePose(geometry_msgs::Pose &pose) {
	addObject(pose);
}

void mapper::addObjects(geometry_msgs::Pose &pose){
	for(auto object : _foundObjects){

	}
}

void mapper:broadcastObject()
*/