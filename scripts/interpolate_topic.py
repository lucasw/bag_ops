#!/usr/bin/env python

import os
import argparse

import rosbag
import rospy

def interpolate_topic(input_bag, target_topic, inter_num):
    input = rosbag.Bag(input_bag, 'r')
    output_bag = os.path.join(os.getcwd(), 'interpolated.bag')
    output = rosbag.Bag(output_bag, 'w')

    if target_topic not in input.get_type_and_topic_info().topics.keys():
        raise ValueError("specified topic does not exist.")

    msg_buffer = []
    for topic, msg, timestamp in input.read_messages():
        if topic == target_topic:
            msg_buffer.append((timestamp.to_sec(), msg))
            if len(msg_buffer) == 2:
                time_step = (msg_buffer[1][0] - msg_buffer[0][0])/(inter_num+1)
                for i in range(1, inter_num+1):
                    output.write(topic, msg_buffer[0][1],
                            rospy.Time.from_sec(msg_buffer[0][0]+i*time_step))
                msg_buffer.pop(0)
        output.write(topic, msg, timestamp)

    output.close()

def main():
    parser = argparse.ArgumentParser(description='''
    interpolate topic (use previous msg between consecutive msgs)
    ''')
    parser.add_argument('input_bag',
            type=str,
            help="input bag file")
    parser.add_argument('target_topic',
            type=str,
            help="topic to shift")
    parser.add_argument('--inter_num',
            type=int,
            help="number of messages to insert into two messages")
    args = parser.parse_args()
    interpolate_topic(args.input_bag, args.target_topic, args.inter_num)

if __name__ == '__main__':
    main()
