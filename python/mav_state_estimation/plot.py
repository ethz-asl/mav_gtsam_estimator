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

import rosbag
import matplotlib.pyplot as plt
import numpy as np

def plot(input_bag, receiver_state_topic):
    topics = [receiver_state_topic]
    bag = rosbag.Bag(input_bag)

    info = bag.get_type_and_topic_info(topics)[1]
    for topic in topics:
        if topic == receiver_state_topic:
            state = np.zeros([info[topic].message_count, 2])

    state_idx = 0

    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == receiver_state_topic:
            if msg.rtk_mode_fix:
                fix = 1
            else:
                fix = 0
            state[state_idx, :] = [msg.header.stamp.to_sec(), fix]
            state_idx += 1

    print("Loaded %d receiver state messages." % (len(state)))

    plt.plot(state[:,0], state[:,1], 'o', label='GNSS Fix')
    plt.legend()
    plt.grid()
    plt.xlabel('Timestamp [s]')
    plt.ylabel('Fix [True/False]')

    plt.show()
