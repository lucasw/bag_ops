/**
 * play back a bag using rosbag_storage bag.h and have ability to change
 * playback rate as well as play backwards, and skip any amount forward or backward
 * in the bag
 */

#include <ros/ros.h>
#include <rosbag/bag.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag_player");

  ros::NodeHandle nh("~");
  std::string bag_name = "";
  nh.getParam("bag", bag_name);
  ROS_INFO_STREAM(bag_name);

  ros::spin();
  return 0;
}
