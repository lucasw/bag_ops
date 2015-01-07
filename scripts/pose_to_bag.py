#!/usr/bin/env python

import os
import argparse

import numpy as np
import bag_ops.transformations as tf
import rosbag
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

def pose_to_bag(file, topic):
    rospy.init_node("pose_to_bag")

    full_name = os.path.realpath(file)
    output_file = os.path.join(os.path.dirname(full_name),
            os.path.splitext(os.path.basename(full_name))[0]+'.bag')
    bag = rosbag.Bag(output_file, 'w')

    pose_data = np.loadtxt(full_name)
    base_sec = rospy.Time.now().to_sec()

    for row in pose_data:
        pose = Pose()
        pose.position = Point(row[1], row[2], row[3])
        pose.orientation = Quaternion(row[5], row[6], row[7], row[4])

        # time
        rel_sec = row[0]
        bag_sec = rospy.Time.from_sec(base_sec+rel_sec)

        # write
        bag.write(topic, pose, bag_sec)

    bag.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Convert plain pose data to bag file
    ''')
    parser.add_argument('file',
            type=str,
            help='file contains pose data')
    parser.add_argument('topic',
            type=str,
            help='topic name to write to')
    args = parser.parse_args()
    pose_to_bag(args.file, args.topic)
