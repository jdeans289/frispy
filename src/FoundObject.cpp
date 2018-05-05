#include "frispy/FoundObject.h"



FoundObject::FoundObject(const int &xC, const int &yC, const float &zC)  : xCenter(xC), yCenter(yC), zCenter(zC) {};

void FoundObject::getDepth(const sensor_msgs::ImageConstPtr& msg) {
  //ROS_INFO("IN GETDEPTH CALLBACK");
	  cv_bridge::CvImagePtr cv_ptr;	
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg); // if no encoding specified, the cv image will have the same encoding as the ros msg
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //ROS_INFO("Rows: %d", cv_ptr->image.rows);
    //ROS_INFO("Cols: %d", cv_ptr->image.cols);
    // int rows = cv_ptr->image.rows;
    // int cols = cv_ptr->image.cols;
    if (detected_box.Class != "bottle")
        return;


    const float THRESH = .3;      // difference threshold
    const float FRACTION = .5; // fraction of bounding box for rectangle radius

     // get width and height of rectangle
    float rectWidth = (detected_box.xmax - detected_box.xmin) * FRACTION;
    float rectHeight = (detected_box.ymax - detected_box.ymin) * FRACTION;



     // get center of box
    xCenter = (detected_box.xmin + detected_box.xmax) / 2;
    yCenter = (detected_box.ymin + detected_box.ymax) / 2;

     // iterate over the mini-rectangle, computing average depth
    float depthSum;
    float depth;
    // float area = rectWidth * rectHeight;
    float numValidPoints = 0;

    ROS_INFO("rectWidth: %f, rectHeight: %f", rectWidth, rectHeight);

    for (int x = xCenter - rectWidth/2; x < xCenter + rectWidth/2; x++) {
      for (int y = yCenter - rectHeight/2; y < yCenter + rectHeight/2; y++) {
        depth = cv_ptr->image.at<float>(cv::Point(x,y));

        // ignore nans and infinities
        if (std::isfinite(depth)) {
          ROS_INFO("finite depth: %f", depth);
          depthSum += depth;
          numValidPoints++;
        }
        // if (depth > zCenter) {
        //     zCenter = depth;
      }
    }
    zCenter = depthSum / numValidPoints;


    // xCenter = (detected_box.xmin + detected_box.xmax) / 2;
  	// yCenter = (detected_box.ymin + detected_box.ymax) / 2;
  	// zCenter = cv_ptr->image.at<float>(cv::Point(xCenter,yCenter));



  	//double depthSum;
  	//int count = (detected_box.xmax - detected_box.xmin) * (detected_box.ymax - detected_box.ymin);

  	// get max depth inside the bounding box
    // for (int x = detected_box.xmin; x < detected_box.xmax; x++) {
    // 	for (int y = detected_box.ymin; y < detected_box.ymax; y++) {
    // 		depthSum += cv_ptr->image.at<double>(cv::Point(x,y));
    // 		//ROS_INFO("%lf", cv_ptr->image.at<float>(cv::Point(x,y)));
    // 		// if (depth > zCenter) {
    // 		// 	zCenter = depth;
    // 	}
    // }

	//zCenter = cv_ptr->image.at<int>(cv::Point(xCenter,yCenter));
	ROS_INFO("Depth at %d,%d: %f", xCenter, yCenter, zCenter);

	buildCube();
   
  	return;
}



void FoundObject::getBox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

  for (int i = 0; i < msg->bounding_boxes.size(); i++) {
  	//ROS_INFO("item name: %s", msg->bounding_boxes[i].Class);
  	// if it's a cup
    if (msg->bounding_boxes[i].Class == "bottle") {
      ROS_INFO("Found the bottle!");
    	detected_box = msg->bounding_boxes[i];

    	break;
    }
  }
  //ROS_INFO ("IN GETBOX CALLBACK");

  return;
}



void FoundObject::buildCube() {




	marker.header.frame_id = "nav_kinect_rgb_optical_frame";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;


	// TFBroadcastPR broadcaster("odom","camera_rgb_optical_frame");
	// mapperPR mapper(broadcaster);

	geometry_msgs::Pose pose;
	pose.position.x = xCenter;
	pose.position.y = yCenter;
	pose.position.z = zCenter;

	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;

	// set scale
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	// set color
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.pose = pose;
	marker.lifetime = ros::Duration();

	return;
}