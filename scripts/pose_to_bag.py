#!/usr/bin/env python

import os
import argparse

import numpy as np
import bag_ops.transformations as tf
import rosbag
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

def pose_to_bag(file):
    rospy.init_node("pose_to_bag")

    full_name = os.path.realpath(file)
    output_file = os.path.join(os.path.dirname(full_name),
            os.path.splitext(os.path.basename(full_name))[0]+'.bag') 

    print full_name, output_file


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Convert plain pose data to bag file
    ''')
    parser.add_argument('file',
            type=str,
            help='file contains pose data')
    args = parser.parse_args()
    pose_to_bag(args.file)
