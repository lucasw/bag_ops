#!/usr/bin/env python

import os
import argparse

import rosbag


def crop_bag(input_bag, crop_time):
    input = rosbag.Bag(input_bag, 'r')
    output_bag = os.path.join(os.getcwd(), 'cropped.bag')
    output = rosbag.Bag(output_bag, 'w')
    start_time = input.get_start_time()
    end_time = input.get_end_time()

    if crop_time > 0:
        start_time += crop_time
    else:
        end_time += crop_time
    if start_time >= end_time:
        raise ValueError("crop time too large")

    for topic, msg, timestamp in input.read_messages():
        if timestamp.to_sec() < start_time or timestamp.to_sec() > end_time:
            continue
        output.write(topic, msg, timestamp)

    output.close()


def main():
    parser = argparse.ArgumentParser(description='''
    crop rosbag file by specified time and direction
    ''')
    parser.add_argument('input_bag',
                        type=str,
                        help="input bag file")
    parser.add_argument('--crop_time',
                        type=float,
                        help="time to crop: positive for cropping from header, negative from tail")
    args = parser.parse_args()
    crop_bag(args.input_bag, args.crop_time)


if __name__ == '__main__':
    main()
