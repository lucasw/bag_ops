#!/usr/bin/env python
import os
import argparse

import rosbag
import rospy

def merge_bags(input_bags, merged_bag):
    rospy.init_node("merge_bags")

    _check_bagfiles(input_bags)

    output_file = os.path.join(os.getcwd(), merged_bag)
    output = rosbag.Bag(output_file, 'w')
    print "Merging", input_bags, "into", output_file

    print "Using the timestamps of the first bag file"
    timestamps = _extract_timestamps(input_bags[0])

    print "Writing merged bag file"
    _write_merged_bag(input_bags, output, timestamps)

    output.close()

def _check_bagfiles(input_bags):
    for bag_file in input_bags:
        if not os.path.exists(bag_file):
            raise ValueError("files not exist")
    bag_objs = []
    for bag in input_bags:
        bag_objs.append(rosbag.Bag(bag))
    for bag in bag_objs:
        if len(bag.get_type_and_topic_info().topics.keys()) != 1:
            raise ValueError("each bag file should contain only one topic")
    if not all(bag.get_message_count() == bag_objs[0].get_message_count() \
            for bag in bag_objs):
        raise ValueError("topics should contain the same number of messages")
    time_cal = lambda b: b.get_end_time()-b.get_start_time()
    if not all(time_cal(bag) == time_cal(bag_objs[0]) for bag in bag_objs):
        raise ValueError("topics should last for the same time length")

def _extract_timestamps(input_bag):
    timestamps = []
    bag = rosbag.Bag(input_bag)
    for _, _, time in bag.read_messages():
        timestamps.append(time)
    return timestamps

def _write_merged_bag(input_bags, output, timestamps):
    for bagfile in input_bags:
        bag = rosbag.Bag(bagfile)
        i = iter(timestamps)
        try:
            for topic, msg, _, in bag.read_messages():
                output.write(topic, msg, i.next())
        except Exception as e:
            print e; raise e

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    merge multiple bag files.
    Each bag file should contain only one topic.
    All topics are of the same time length and time duration.
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
