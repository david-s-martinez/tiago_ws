
#include <plane_segmentation/plane_segmentation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segmentation");
  ros::NodeHandle nh;
  
  //#>>>>TODO: Set the correct topic name of the robot
  std::string pointcloud_topic_name = "/xtion/depth_registered/points";
  //rgb topic: /xtion/rgb/image_raw
  //#>>>>TODO: Set the name of a frame on the floor/ground of the robot (height=0)
  std::string base_fame_name = "base_footprint";

  // construct the object
  PlaneSegmentation segmentation(
    pointcloud_topic_name, 
    base_fame_name);
  
  // initialize the object
  if(!segmentation.initalize(nh))
  {
    ROS_ERROR_STREAM("Error init PlaneSegmentation");
    return -1;
  }

  // update the processing
  ros::Rate rate(10);
  while(ros::ok())
  {
    segmentation.update(ros::Time::now());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
