#!/usr/bin/env python
# print out the first num_bytes in a 'data' field of all messages in a bag

import sys

import rosbag
import rospy


if __name__ == "__main__":
    rospy.init_node("bag_data_bytes")
    argv = [arg for arg in sys.argv[1:] if not arg.startswith("_")]

    num_bytes_to_print = rospy.get_param("~num_bytes", 24)

    for input_bag in argv:
        rospy.loginfo(f"opening {input_bag}")
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if rospy.is_shutdown():
                break
            if hasattr(msg, 'data'):
                text = ""  # f"{t.to_sec():0.3f} {topic}:"
                for ind, byte in enumerate(msg.data):
                    if ind >= num_bytes_to_print:
                        break
                    text += f" {byte:02x}"
                    if ind % 8 == 7:
                        text += " "
                print(text)
