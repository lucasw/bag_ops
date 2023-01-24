#!/usr/bin/env python
# Lucas Walter
# output the bytes in a 'data' field of all messages in a bag on a given topic to a file

import sys

import rosbag
import rospy


def get_bytes(input_bag, out_file, selected_topic, verbose=False, num_bytes_to_print=0):
    rospy.loginfo(f"opening {input_bag} {out_file} {selected_topic} {num_bytes_to_print}")
    count = 0
    num_bytes = 0
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if topic is not None and topic != selected_topic:
            # rospy.loginfo_throttle(0.0, f"{topic} != {selected_topic}")
            continue
        if rospy.is_shutdown():
            break
        data = None
        if hasattr(msg, 'data'):
            data = msg.data
        elif hasattr(msg, 'value'):
            value = msg.value
            text = ""
            data = value.to_bytes(4, "big")
            for byte in data:
                text += f" 0x{byte:02x}"
            text += f" {value}"
            # print(text)
        if data is not None:
            out_file.write(data)
            num_bytes += len(data)
            count += 1
            rospy.loginfo_throttle(2.0, f"{num_bytes}")
            # print(f"test {num_bytes_to_print}")
            if num_bytes_to_print > 0:
                text = ""  # f"{t.to_sec():0.3f} {topic}:"
                for ind, byte in enumerate(data):
                    if ind >= num_bytes_to_print:
                        break
                    text += f" 0x{byte:02x}"
                    if ind % 8 == 7:
                        text += " "
                rospy.loginfo_throttle(1.0, text)
    rospy.loginfo(f"{num_bytes} written from {count} messages")
    return num_bytes


if __name__ == "__main__":
    rospy.init_node("bag_data_bytes")
    topic = rospy.get_param("~topic", "")
    num_bytes_to_print = rospy.get_param("~num_bytes", 0)
    print(num_bytes_to_print)
    out_filename = rospy.get_param("~out_filename", "bytes.bin")
    argv = [arg for arg in sys.argv[1:] if not arg.startswith("_")]

    with open(out_filename, "wb") as out_file:
        rospy.loginfo(f"writing to {out_filename}")
        for input_bag in argv:
            get_bytes(input_bag, out_file, topic, num_bytes_to_print=num_bytes_to_print)
