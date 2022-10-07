#!/usr/bin/env python
# rewrite bag with header timestamps
# from http://wiki.ros.org/rosbag/Cookbook#Rewrite_bag_with_header_timestamps
import sys

import rosbag

input_bag = sys.argv[1]
output_bag = sys.argv[2]

with rosbag.Bag(output_bag, "w") as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
