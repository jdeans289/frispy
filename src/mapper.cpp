#include "frispy/mapper.h"

mapper::mapper(TFBroadcastPR &br) : _broadcaster(br), _foundObjects() {}

//Recieves pose and object class string
void mapper::storeObject(const frispy::object &object) {
	 std::vector<geometry_msgs::Pose> newList();
	 newList.push_back(object.location);
	_foundObjects.insert({object.Class, newList);
}

void mapper::broadcastAllObjects() {
	for(auto& object : _foundObjects) {
		//broadcasts the pose of every object in _foundObjects
		for(auto &location : object.second)	
			_broadcaster.receivePose(location);
	}
}

void mapper::broadcastSelectedObjects(std::vector<std::string> &objects) {
				//is this supposed to be objects, not object?
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