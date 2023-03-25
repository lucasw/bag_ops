#!/usr/bin/env python
# Lucas Walter

# read topics with Headers from a bag and interpolate them using tf BufferCore
# and output a new bag with all the topics synchronized and published at a set rate

import sys

import rosbag
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def to_buffer_core(buffer_core, value, stamp, parent_frame, child_frame):
    ts = TransformStamped()
    ts.header.frame_id = parent_frame
    ts.header.stamp = stamp
    # in tf2 frame_ids cannot start with a '/'
    ts.child_frame_id = child_frame
    # TODO(lucasw) only works with messages with a value field,
    # like marti_common_msgs
    ts.transform.translation.x = value
    ts.transform.rotation.w = 1.0
    buffer_core.set_transform(ts, "default_authority")


period = float(sys.argv[1])
input_bag = sys.argv[2]
output_bag = sys.argv[3]
topics = sys.argv[4:]

print(topics)
print(f"sync these topics from {input_bag}: {topics} with period {period:0.2f}s")

t_min = {}
t_max = {}
for topic in topics:
    t_min[topic] = None
    t_max[topic] = None

base_frame = "base"

frames = {}

# TODO(lucasw) get duration from input_bag
duration = 8000.0
buffer_core = tf2_ros.BufferCore(rospy.Duration(duration))
count = 0
with rosbag.Bag(input_bag) as bag:
    topic_info = bag.get_type_and_topic_info().topics
    for topic in topics:
        print(topic_info[topic])
    for topic, msg, t in bag.read_messages():
        if topic not in topics:
            continue
        stamp = msg.header.stamp
        if t_min[topic] is None:
            t_min[topic] = stamp
        t_min[topic] = min(stamp, t_min[topic])
        if t_max[topic] is None:
            t_max[topic] = stamp
        t_max[topic] = max(stamp, t_max[topic])

        if topic_info[topic].msg_type == "sensor_msgs/JointState":
            for values, postfix in zip([msg.position, msg.velocity], ["position", "velocity"]):
                for name, value in zip(msg.name, values):
                    frame = topic[1:] + "_" + name + "_" + postfix
                    frames[frame] = True
                    to_buffer_core(buffer_core, value, stamp, base_frame, frame)
        else:
            frame = topic[1:]
            # TODO(lucasw) check if msg has 'value'
            try:
                to_buffer_core(buffer_core, msg.value, stamp, base_frame, frame)
                frames[frame] = True
            except AttributeError as ex:
                print(topic_info[topic].msg_type)
                raise(ex)
            if count % 2000 == 0:
                print(f"{count}")
            count += 1
        if False:  # count > 200:
            print(f"breaking early after {count} transforms set")
            break

print(frames.keys())

t_start = max([t_min[topic] for topic in t_min.keys()])
# round start up
t_start.secs += 1
t_start.nsecs = 0

t_end = min([t_max[topic] for topic in t_max.keys()])
# round end down
t_end.nsecs = 0

duration = (t_end - t_start).to_sec()
num_samples = int(duration / period)
print(f"{t_start.to_sec():0.2f}s to {t_end.to_sec():0.2f}s {duration:0.1f}s {num_samples} samples")

with rosbag.Bag(output_bag, "w") as outbag:
    for i in range(num_samples):
        tfm = TFMessage()
        stamp = t_start + rospy.Duration(i * period)
        for frame in frames.keys():
            # TODO(lucasw) write every message as a TransformStamped for now, later use the input topic type
            # though it is convenient to put all the topics together in one message, easier
            # to iterate through downstream
            try:
                tf = buffer_core.lookup_transform_core(base_frame, frame, stamp)
            except Exception as ex:
                rospy.logwarn(ex)
                continue
            tfm.transforms.append(tf)
        outbag.write("sync_topics_tf", tfm, stamp)
