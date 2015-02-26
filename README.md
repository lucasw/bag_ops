# bag_ops
This package contains some functions to operate on bag files.

# list
* pose_to_bag: convert plain pose data to bag file
* merge_bags: merge and align several bags that contain only one topic each. These topics should of the same time duration and message number
* crop_bags: crop bag file from head or tail for a certain time duration
* shift_topic: shift the timestamps of a certain topic forward or backward
* interpolate_topic: insert a certain number of messages between two consecutive messages, using the value of the previous one
