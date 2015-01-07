#!/usr/bin/env python
import os
import argparse

import rosbag
import rospy

def merge_bags(input_bags, merged_bag):
    rospy.init_node("merge_bags")

    _check_bagfiles(input_bags)

    output_file = os.path.join(os.getcwd(), merged_bag)

    bag_objs = []
    for bag in input_bags:
        bag_objs.append(rosbag.Bag(bag))

    print "Merging", input_bags, "into", output_file
    output = rosbag.Bag(output_file, 'w')

    output.close()


def _check_bagfiles(input_bags):
    for bag in input_bags:
        if not os.path.exists(bag):
            raise ValueError("Files not exist")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    merge multiple bag files
    ''')
    parser.add_argument('input_bags',
            type=str,
            nargs='*',
            help='bag files list')
    parser.add_argument('--merged_bag',
            type=str,
            default='merged.bag',
            help="name of merged bag file"
            )
    args = parser.parse_args()
    merge_bags(args.input_bags, args.merged_bag)
