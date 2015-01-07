#!/usr/bin/env python

import os
import argparse

import numpy as np
import bag_ops.transformations as tf
import rosbag
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

def pose_to_bag(file, topic, axies):
    rospy.init_node("pose_to_bag")

    full_name = os.path.realpath(file)
    output_file = os.path.join(os.path.dirname(full_name),
            os.path.splitext(os.path.basename(full_name))[0]+'.bag')
    bag = rosbag.Bag(output_file, 'w')

    pose_data = np.loadtxt(full_name)
    base_sec = rospy.Time.now().to_sec()

    try:
        _check_data(pose_data)
        for row in pose_data:
            pose = _row_to_pose(row, axies)
            bag_sec = rospy.Time.from_sec(base_sec+row[0])
            bag.write(topic, pose, bag_sec)
    except Exception as e:
        raise e

    bag.close()

# the order of transformation output: w x y z
# the order of ROS: x y z w
def _row_to_pose(row, axies):
    pose = Pose()
    pose.position = Point(row[1], row[2], row[3])
    if row.shape[0] == 8:
        pose.orientation = Quaternion(row[4], row[5], row[6], row[7])
    elif row.shape[0] == 7:
        if axies == '':
            raise ValueError('The type of euler should be specified')
        quat = tf.quaternion_from_euler(row[4], row[5], row[6], axies)
        pose.orientation = Quaternion(quat[1], quat[2], quat[3], quat[0])
    else:
        raise TypeError('The lenght of the array should be 7 or 8')
    return pose

def _check_data(pose_data):
    shape = pose_data.shape
    if shape[1] != 7 and shape[1] != 8:
        raise TypeError('The width of the array should be 7 or 8')

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
    parser.add_argument('--axies',
            type=str,
            default='',
            help="""if use euler angle, specify the type 
              Axes 4-string: e.g. 'sxyz' or 'ryxy'
              first character : rotations are applied to 's'tatic or 'r'otating frame
              remaining characters : successive rotation axis 'x', 'y', or 'z'
            """
            )
    args = parser.parse_args()
    pose_to_bag(args.file, args.topic, args.axies)
