#!/usr/bin/env python

import os
import argparse

import numpy as np
import bag_ops.transformations as tf
import rosbag
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

def pose_to_bag(file):
    print 'pose_to_bag'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Convert plain pose data to bag file
    ''')
    parser.add_argument('file',
            type=str,
            help='file contains pose data')
    args = parser.parse_args()
    pose_to_bag(args.file)
