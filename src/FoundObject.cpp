#include "frispy/FoundObject.h"



FoundObject::FoundObject(const int &xC, const int &yC, const float &zC)  : xCenter(xC), yCenter(yC), zDepth(zC) {};




void FoundObject::getBox(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

  for (int i = 0; i < msg->bounding_boxes.size(); i++) {
    //ROS_INFO("item name: %s", msg->bounding_boxes[i].Class);
    // if it's a cup
    if (msg->bounding_boxes[i].Class == "person" || msg->bounding_boxes[i].Class == "bottle" || msg->bounding_boxes[i].Class == "cup") {
      ROS_INFO("Found the bottle!");
      detected_box = msg->bounding_boxes[i];

      break;
    }
  }
  return;
}



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
    if (detected_box.Class != "person" && detected_box.Class != "vase" && detected_box.Class != "cup")
        return;


    // get center of box
    xCenter = (detected_box.xmin + detected_box.xmax) / 2;
    yCenter = (detected_box.ymin + detected_box.ymax) / 2;


    // *** CENTER DEPTH APPROACH ***

    // zDepth = cv_ptr->image.at<float>(cv::Point(xCenter,yCenter));


    // *** FRACTIONAL RECTANGLE APPROACH ***


    const float THRESH = .3;      // difference threshold
    const float FRACTION = .5; // fraction of bounding box for rectangle radius

     // get width and height of rectangle
    float rectWidth = (detected_box.xmax - detected_box.xmin) * FRACTION;
    float rectHeight = (detected_box.ymax - detected_box.ymin) * FRACTION;


    // iterate over the mini-rectangle, computing average depth
    float depthSum;
    float depth;
    // float area = rectWidth * rectHeight;
    float numValidPoints = 0;



    for (int x = xCenter - rectWidth/2; x < xCenter + rectWidth/2; x++) {
      for (int y = yCenter - rectHeight/2; y < yCenter + rectHeight/2; y++) {
        depth = cv_ptr->image.at<float>(cv::Point(x,y));

        // ignore nans and infinities
        if (std::isfinite(depth)) {
          //ROS_INFO("finite depth: %f", depth);
          depthSum += depth;
          numValidPoints++;
        }
        // if (depth > zCenter) {
        //     zCenter = depth;
      }
    }
    zDepth = depthSum / numValidPoints;

  	

  	// *** MAX DEPTH APPROACH ***

    // for (int x = detected_box.xmin; x < detected_box.xmax; x++) {
    // 	for (int y = detected_box.ymin; y < detected_box.ymax; y++) {
    // 		depthSum += cv_ptr->image.at<double>(cv::Point(x,y));
    // 		//ROS_INFO("%lf", cv_ptr->image.at<float>(cv::Point(x,y)));
    // 		// if (depth > zCenter) {
    // 		// 	zCenter = depth;
    // 	}
    // }

   //ROS_INFO("xmax-xmin: %ld | ymax-ymin: %ld", detected_box.xmax - detected_box.xmin, detected_box.ymax - detected_box.ymin);

	 //ROS_INFO("representative depth with center %d,%d: %f", xCenter, yCenter, zDepth);
   // ROS_INFO("getDepth time: %d", ros::Time::now());
   
  	return;
}



/* using the x y pixels from the FoundObject class, get the associated coordinates */
void FoundObject::getLocation(const sensor_msgs::PointCloud2& msg) {

  // *** thank you to Saurav Argawal for this solution ***


  //ROS_INFO("xCenter (pixel): %d | yCenter (pixel): %d", xCenter, yCenter);
  int pointIndex, xIndex, yIndex, zIndex; // indeces of x, y, and z coordinates
  pointIndex = yCenter * msg.row_step + xCenter * msg.point_step;
  xIndex = pointIndex;
  yIndex = pointIndex + msg.fields[1].offset; 
  // zIndex = pointIndex + msg.fields[2].offset; // dont need because we're using the depth image directly


  //ROS_INFO("height: %d", msg.height);
  //ROS_INFO("width: %d", msg.width);

  float x, y; // z;

  memcpy(&x, &msg.data[xIndex], sizeof(float));
  memcpy(&y, &msg.data[yIndex], sizeof(float));
  //memcpy(&z, &msg.data[zIndex], sizeof(float));

  geometry_msgs::Point objectLocation;

  objectLocation.x = x;
  objectLocation.y = y;
  objectLocation.z = zDepth;


  // ROS_INFO("32 bit XYZ: %f, %f, %f", x, y, z);
  

  // transform our new point relative to odom


  tf::TransformListener listener;
  // tf::Stamped<tf::Point> original;
  // tf::Stamped<tf::Point> final;
  geometry_msgs::PointStamped ogLocation;

  // pack our location into a tf::PointStamped
  ogLocation.header.stamp = ros::Time();
  ogLocation.header.frame_id = "/nav_kinect_rgb_optical_frame";
  ogLocation.point = objectLocation;
  


  tf::StampedTransform testTransform;

  try {
    listener.waitForTransform("/odom", "/nav_kinect_rgb_optical_frame", ros::Time(0), ros::Duration(4));
    //listener.lookupTransform("/odom", "/nav_kinect_rgb_optical_frame", ros::Time(0), testTransform);
    //ROS_INFO("nav_kinect -> odom: %lf, %lf, %lf", testTransform.getOrigin().getX(), testTransform.getOrigin().getY(), testTransform.getOrigin().getZ() );

    listener.transformPoint("/odom", ogLocation, finalLocation);
    // works until here
  } catch (tf::TransformException ex) {}

  ROS_INFO("ogLocation XYZ: %lf, %lf, %lf", ogLocation.point.x, ogLocation.point.y, ogLocation.point.z);

  // the current problem is that the transformed coordinates (relative to odom) are all 0!!! How to fix???


  buildCube();
} 




void FoundObject::buildCube() {

	marker.header.frame_id = "/odom";
	marker.header.stamp = ros::Time();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

  // set scale
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // set color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;


	// TFBroadcastPR broadcaster("odom","camera_rgb_optical_frame");
	// mapperPR mapper(broadcaster);

	geometry_msgs::Pose finalPose;

  
  // hard code the point for now
  //geometry_msgs::Point hardPoint;
  // hardPoint.x = 1.0;
  // hardPoint.y = 1.0;
  // hardPoint.z = 1.0;

  ROS_INFO("finalLocation XYZ: %lf, %lf, %lf", finalLocation.point.x, finalLocation.point.y, finalLocation.point.z);


  // put the calculated point into the pose
  finalPose.position = finalLocation.point;

	finalPose.orientation.x = 0;
	finalPose.orientation.y = 0;
	finalPose.orientation.z = 0;
	finalPose.orientation.w = 1;



	marker.pose = finalPose;
	marker.lifetime = ros::Duration();

	return;
}
