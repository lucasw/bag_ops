#!/usr/bin/env python
import os
import argparse

import rosbag
import rospy

def merge_bags(input_bags, merged_bag):
    rospy.init_node("merge_bags")

    for bag in input_bags:
        try:
            _check_bag_file(bag)
        except Exception as e:
            raise e
    output_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
            merged_bag)

    print "Merging", input_bags, "into", merged_bag 


def _check_bag_file(bag):
    if not os.path.exists(bag):
        raise ValueError("Files not exist")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    merge multiple bag files
    ''')
    parser.add_argument('input_bags',
            type=str,
            nargs='*',
            help='file contains pose data')
    parser.add_argument('--merged_bag',
            type=str,
            default='merged.bag',
            help="name of merged bag file"
            )
    args = parser.parse_args()
    merge_bags(args.input_bags, args.merged_bag)
