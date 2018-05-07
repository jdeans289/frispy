#include "frispy/mapper.h"

mapper::mapper(TFBroadcastPR &br) : _broadcaster(br), _foundObjects() {}

float mapper::distanceBetween(geometry_msgs::Point p1, geometry_msgs::Point p2) {
	auto diffX = p2.x - p1.x;
	auto diffY = p2.y - p1.y;
	auto diffZ = p2.z - p1.z;
	return std::sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
}
//Recieves pose and object class string
void mapper::storeObject(const frispy::object &object) {
<<<<<<< HEAD
	std::vector<geometry_msgs::Pose> *previouslyFound;
	
	if(_foundObjects.find(object.Class) != _foundObjects.end()) {
		previouslyFound = &_foundObjects.find(object.Class)->second;
		ROS_INFO("Already have %s", object.Class.c_str());		
		for(auto pose : *previouslyFound) {
			// ROS_INFO("Distance is %f", distanceBetween(pose.position, object.location.pose.position));		
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
=======
	 std::vector<geometry_msgs::Pose> newList();
	 newList.push_back(object.location);
	_foundObjects.insert(object.Class, newList);
>>>>>>> 56d2c76a9fb9f5d651d74c0a595e236cd35c3e3e
}

void mapper::broadcastAllObjects() {
	for(auto& object : _foundObjects) {
		//broadcasts the pose of every object in _foundObjects
		for(auto &location : object.second) {
			_broadcaster.receivePose(location);
		}
	}
}

void mapper::broadcastSelectedObjects(std::vector<std::string> &objects) {
	for(auto &objectType : objects) {
		auto match = _foundObjects.find(objectType);
		if(match != _foundObjects.end()) {
			for(auto &location : match->second)
				_broadcaster.receivePose(location);
		}
		else {
			// ROS_INFO("No " + objectType + "s have been found.");
		}
	}
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