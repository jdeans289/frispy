#include "frispy/FoundObject.h"

FoundObject::FoundObject(const int &xC, const int &yC, const float &zC, ros::Publisher &pub) : 
                        xCenter(xC), yCenter(yC), zDepth(zC), object_pub(pub) {};

/* Callback invoked when bounding boxes are reveived from YOLO */
void FoundObject::processBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

  // grab single messages for the depth image and the point cloud
  const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/nav_kinect/depth_registered/points", 
                                                                  ros::Duration(5)); 
  const sensor_msgs::ImageConstPtr& depthImageMsg = ros::topic::waitForMessage<sensor_msgs::Image>("/nav_kinect/depth_registered/hw_registered/image_rect", 
                                                                  ros::Duration(5));

  // operate on each bounding box received from yolo, if the probability is high enough
  for (int i = 0; i < msg->bounding_boxes.size(); i++) {
    if (msg->bounding_boxes[i].probability > 0.5) {
      
      detected_box = msg->bounding_boxes[i];

      // process away!
      getDepth(depthImageMsg);
      getLocation(pointCloudMsg);
      //getDimensions(pointCloudMsg);

      // populate our custom object with detectedObject class and location.
      thisObject.Class = detected_box.Class;
      thisObject.location.pose.position = finalLocation.point;
      thisObject.location.header.frame_id = "/odom";
      thisObject.height = 0.1;
      thisObject.width = 0.1;

      // as long as we don't have nan values, send it off!
      if (!std::isnan(thisObject.location.pose.position.x)) {
        ROS_INFO("Detected %s at %f %f %f", thisObject.Class.c_str(), thisObject.location.pose.position.x, 
          thisObject.location.pose.position.y, thisObject.location.pose.position.z);
        object_pub.publish(thisObject);
      }
    }
  }

  return;
}

void FoundObject::getDepth(const sensor_msgs::ImageConstPtr& msg) {
    //ROS_INFO("IN GETDEPTH")
	  cv_bridge::CvImagePtr cv_ptr;	
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // get center of box
    xCenter = (detected_box.xmin + detected_box.xmax) / 2;
    yCenter = (detected_box.ymin + detected_box.ymax) / 2;


    // *** FRACTIONAL RECTANGLE APPROACH ***


    const float THRESH = .3;      // difference threshold
    const float FRACTION = .5;    // fraction of bounding box for rectangle radius

     // get width and height of rectangle
    float rectWidth = (detected_box.xmax - detected_box.xmin) * FRACTION;
    float rectHeight = (detected_box.ymax - detected_box.ymin) * FRACTION;


    float depthSum;
    float depth;
    float numValidPoints = 0;

    // iterate over the mini-rectangle, computing average depth
    for (int x = xCenter - rectWidth/2; x < xCenter + rectWidth/2; x++) {
      for (int y = yCenter - rectHeight/2; y < yCenter + rectHeight/2; y++) {
        depth = cv_ptr->image.at<float>(cv::Point(x,y));

        // ignore nans and infinities
        if (std::isfinite(depth)) {
          depthSum += depth;
          numValidPoints++;
        }
      }
    }
    zDepth = depthSum / numValidPoints;
   
  	return;
}

/* using the x y pixels from the FoundObject class, get the associated coordinates */
void FoundObject::getLocation(const sensor_msgs::PointCloud2ConstPtr& msg) {

  // *** thank you to Saurav Argawal for this solution ***

  int pointIndex, xIndex, yIndex, zIndex; // indeces of x, y, and z coordinates

  xIndex = getPointCloudXCoordinate(msg, xCenter, yCenter);
  yIndex = getPointCloudYCoordinate(msg, xCenter, yCenter);

  float x, y; // z;

  memcpy(&x, &msg->data[xIndex], sizeof(float));
  memcpy(&y, &msg->data[yIndex], sizeof(float));
  //ROS_INFO("Real World: xCenter: %f\tyCenter: %f\n", x, y);

  geometry_msgs::Point objectLocation;

  objectLocation.x = x;
  objectLocation.y = y;
  objectLocation.z = zDepth;

  // transform our new point relative to odom
  tf::TransformListener listener;
  geometry_msgs::PointStamped ogLocation;

  // pack our location into a tf::PointStamped
  ogLocation.header.stamp = ros::Time();
  ogLocation.header.frame_id = "/nav_kinect_rgb_optical_frame";
  ogLocation.point = objectLocation;

  try {
    listener.waitForTransform("/odom", "/nav_kinect_rgb_optical_frame", ros::Time(0), ros::Duration(4));
    listener.transformPoint("/odom", ogLocation, finalLocation);
  } catch (tf::TransformException ex) {}

  ROS_INFO("ogLocation XYZ: %lf, %lf, %lf", ogLocation.point.x, ogLocation.point.y, ogLocation.point.z);

  return;
} 

// Helper method to do the nasty point cloud calculations for x coordinates
int FoundObject::getPointCloudXCoordinate (const sensor_msgs::PointCloud2ConstPtr& msg, int x, int y) {
  return y * msg->row_step + x * msg->point_step + msg->fields[0].offset;
}

// Helper method to do the point cloud for y coordinates
int FoundObject::getPointCloudYCoordinate (const sensor_msgs::PointCloud2ConstPtr& msg, int x, int y) {
  return y * msg->row_step + x * msg->point_step + msg->fields[1].offset;
}

// An attempt to determine width and height of the objects themselves based off of bounding
// boxes. Will be revisited later

/* 
void FoundObject::getDimensions (const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_INFO("Yolo bounding boxes\nXmin: %d\tXmax:%d\nYmin: %d\tYmax:%d\n", detected_box.xmin, detected_box.xmax, detected_box.ymin, detected_box.ymax);
  int yMaxIndex = getPointCloudYCoordinate(msg, xCenter, detected_box.ymax);
  int yMinIndex = getPointCloudYCoordinate(msg, xCenter, detected_box.ymin);
  int xMaxIndex = getPointCloudXCoordinate(msg, detected_box.xmax, yCenter);
  int xMinIndex = getPointCloudXCoordinate(msg, detected_box.xmin, yCenter);
  ROS_INFO("Point Cloud boundaries boxes\nXmin: %d\tXmax:%d\nYmin: %d\tYmax:%d\n", xMinIndex, xMaxIndex, yMinIndex, yMaxIndex);


  float yMax, yMin, xMax, xMin;
  yMax = yMin = xMax = xMin = 0;
  getXCoord(&xMin, msg, detected_box.xmin, yCenter, sizeof(float));// memcpy(&xMin, &msg->data[xMinIndex], sizeof(float));
  getXCoord(&xMin, msg, detected_box.xmax, yCenter, sizeof(float));// memcpy(&xMax, &msg->data[xMaxIndex], sizeof(float));
  memcpy(&yMin, &msg->data[yMinIndex], sizeof(float));
  memcpy(&yMax, &msg->data[yMaxIndex], sizeof(float));

  ROS_INFO("xMin: %f\txMax: %f\nyMin: %f\tyMax%f\n", xMin, xMax, yMin, yMax);

  thisObject.height = yMax - yMin;
  ROS_INFO("Real world ymin and ymax: %f - %f", yMin, yMax);
  thisObject.width = xMax - xMin;
  ROS_INFO("Real world xmin and xmax: %f - %f", xMin, xMax);

}
*/

// *** OLD BUILD CUBE METHOD, FOR VISUALIZATION PURPOSES

/*
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
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

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


*/ 
