/**
 * Copyright 2023 Lucas Walter
 * play back a bag using rosbag_storage bag.h and have ability to change
 * playback rate as well as play backwards, and skip any amount forward or backward
 * in the bag
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <string>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bag_player");

  ros::NodeHandle nh("~");
  std::string bag_name = "";
  nh.getParam("bag", bag_name);
  ROS_INFO_STREAM("opening bag: '" << bag_name << "'");

  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Read);

  bag.close();

  ros::spin();
  return 0;
}
