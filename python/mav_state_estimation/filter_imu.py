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
import collections
import numpy as np

# See https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16448.pdf
def filterImu(inbag, outbag, imu_topic, bartlett_filter_size = 0):
    imu_filtered = imu_topic + '_filtered'

    N_B = 2 ** bartlett_filter_size
    deque1 = collections.deque(maxlen=N_B)
    deque2 = collections.deque(maxlen=N_B)
    stamps = collections.deque(maxlen=N_B)

    with rosbag.Bag(outbag, 'w') as bag:
        for topic, msg, t in rosbag.Bag(inbag).read_messages():
            if (msg._has_header and msg.header.stamp.secs == 0):
                continue
            bag.write(topic, msg, t)
            if (topic == imu_topic):
                # First stage
                deque1.append(np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]))
                x1 = np.average(np.stack(deque1), axis=0)
                # Second stage
                deque2.append(x1)
                x2 = np.average(np.stack(deque2), axis=0)
                stamps.append(msg.header.stamp)
                filtered_msg = msg
                filtered_msg.header.stamp = stamps[0]
                filtered_msg.angular_velocity.x = x2[0]
                filtered_msg.angular_velocity.y = x2[1]
                filtered_msg.angular_velocity.z = x2[2]
                filtered_msg.linear_acceleration.x = x2[3]
                filtered_msg.linear_acceleration.y = x2[4]
                filtered_msg.linear_acceleration.z = x2[5]
                bag.write(imu_filtered, filtered_msg, t)
