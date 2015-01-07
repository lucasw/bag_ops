#!/usr/bin/env python
import os
import argparse

import rosbag
import rospy

def merge_bags(input_bags, merged_bag):
    print "Merging", input_bags, "into", merged_bag 

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
