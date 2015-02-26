#!/usr/bin/env python

import os
import argparse

import rosbag
import rospy

def shift_topic(input_bag, target_topic, shift_time):
    input = rosbag.Bag(input_bag, 'r')
    output_bag = os.path.join(os.getcwd(), 'shiftted.bag')
    output = rosbag.Bag(output_bag, 'w')

    if target_topic not in input.get_type_and_topic_info().topics.keys():
        raise ValueError("specified topic does not exist.")

    for topic, msg, timestamp in input.read_messages():
        if topic == target_topic:
            timestamp = rospy.Time.from_sec(timestamp.to_sec()+shift_time)
        output.write(topic, msg, timestamp)

    output.close()

def main():
    parser = argparse.ArgumentParser(description='''
    shift one of the topics in bag file forward or backward in time
    ''')
    parser.add_argument('input_bag',
            type=str,
            help="input bag file")
    parser.add_argument('target_topic',
            type=str,
            help="topic to shift")
    parser.add_argument('--shift_time',
            type=float,
            help="time to shift: positive for shift forward, \
                    negative from backward")
    args = parser.parse_args()
    shift_topic(args.input_bag, args.target_topic, args.shift_time)

if __name__ == '__main__':
    main()
