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

# TODO(lucasw) get duration from input_bag
duration = 1000.0
buffer_core = tf2_ros.BufferCore(rospy.Duration(duration))
count = 0
for topic, msg, t in rosbag.Bag(input_bag).read_messages():
    if topic not in topics:
        continue
    stamp = msg.header.stamp
    if t_min[topic] is None:
        t_min[topic] = stamp
    t_min[topic] = min(stamp, t_min[topic])
    if t_max[topic] is None:
        t_max[topic] = stamp
    t_max[topic] = max(stamp, t_max[topic])

    ts = TransformStamped()
    ts.header.frame_id = base_frame
    ts.header.stamp = stamp
    # in tf2 frame_ids cannot start with a '/'
    ts.child_frame_id = topic[1:]
    # TODO(lucasw) only works with messages with a value field,
    # like marti_common_msgs
    ts.transform.translation.x = msg.value
    ts.transform.rotation.w = 1.0
    buffer_core.set_transform(ts, "default_authority")
    if count % 2000 == 0:
        print(f"{count}")
    count += 1
    if False:  # count > 200:
        print(f"breaking early after {count} transforms set")
        break

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
        for topic in topics:
            # TODO(lucasw) write every message as a TransformStamped for now, later use the input topic type
            # though it is convenient to put all the topics together in one message, easier
            # to iterate through downstream
            tf = buffer_core.lookup_transform_core(base_frame, topic[1:], stamp)
            tfm.transforms.append(tf)
        outbag.write("sync_topics_tf", tfm, stamp)
