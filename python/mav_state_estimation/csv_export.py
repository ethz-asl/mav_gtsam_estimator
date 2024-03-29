#!/usr/bin/python

# MIT License
#
# Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import rosbag
import rospy
from datetime import datetime

def getStamp(stamp):
    dt = datetime.utcfromtimestamp(stamp.secs)
    us = stamp.nsecs // 1e3
    us_mod = stamp.nsecs % 1e3
    return "%d,%d,%d,%d,%d,%d,%d.%d" % (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, us, us_mod)

def getTranslation(translation):
    return "%f,%f,%f" %(translation.x, translation.y, translation.z)

def getRotation(rotation):
    return "%f,%f,%f,%f" %(rotation.w, rotation.x, rotation.y, rotation.z)


def toCsv(bag_file, topics, stamp_topic=None, feedback=None):

    if feedback:
        from mav_state_estimation.msg import BatchStatus
        status = BatchStatus()
        status.finished = False
        status.current_idx = 0
        status.total_idx = 0

    if not os.path.exists(bag_file):
        rospy.logerr("File %s does not exist." % (bag_file))
        return False

    if not os.path.isfile(bag_file):
        rospy.logerr("File %s is not a file." % (bag_file))
        return False


    csv_path = os.path.dirname(bag_file) + '/export/'
    if not os.path.exists(csv_path):
        os.mkdir(csv_path)

    bag = rosbag.Bag(bag_file)

    f = {}
    info = bag.get_type_and_topic_info(topics)[1]
    for topic in topics:
        if topic in info.keys() and info[topic].message_count > 0 and info[topic].msg_type == 'geometry_msgs/TransformStamped':
            csv_file = csv_path + topic + '.csv'
            rospy.loginfo("Creating new CSV file %s" % (csv_file))
            f[topic] = open(csv_file, 'w')
            f[topic].write('year, month, day, hour, min, second, microsecond, x, y, z, qw, qx, qy, qz\n')

            if feedback:
                status.total_idx = status.total_idx + info[topic].message_count

    stamps = set()
    if stamp_topic:
        for topic, msg, t in bag.read_messages(topics=[stamp_topic]):
            stamps.add(msg.header.stamp)

    for topic, msg, t in bag.read_messages(topics=f.keys()):
        if msg.header.stamp and msg.transform.translation and msg.transform.rotation and f[topic]:

            if feedback:
                status.current_idx = status.current_idx + 1
                if (10 * status.current_idx) % status.total_idx == 0:
                    feedback.publish(status) # Every 10 percent

            if stamp_topic and msg.header.stamp not in stamps:
                continue

            f[topic].write(getStamp(msg.header.stamp))
            f[topic].write(",")
            f[topic].write(getTranslation(msg.transform.translation))
            f[topic].write(",")
            f[topic].write(getRotation(msg.transform.rotation))
            f[topic].write("\n")

    for topic in f:
        f[topic].close()

    if feedback:
        status.finished = True
        feedback.publish(status)
    return True
