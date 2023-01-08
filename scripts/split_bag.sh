#!/bin/bash
# Lucas Walter
# provide input bag, output bags prefix, and time fraction
#
# https://answers.ros.org/question/99711/how-to-split-a-recorded-rosbag-file/?answer=290154#post-id-290154
echo $1, $2, $3
t0=`rosbag info -y -k start $1`
t1=`rosbag info -y -k end $1`
tfr=`echo "$t0 + ($t1 - $t0) * $3" | bc -l`
echo $t0, $t1, $tfr
rosbag filter $1 $2_a.bag "t.secs <= $tfr"
rosbag filter $1 $2_b.bag "t.secs > $tfr"
