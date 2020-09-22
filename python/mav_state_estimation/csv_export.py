import os
import rosbag
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


def toCsv(bag_file, topics):
    csv_path = os.path.dirname(bag_file) + '/export/'
    if not os.path.exists(csv_path):
        os.mkdir(csv_path)

    f = {}
    for topic in topics:
        csv_file = csv_path + topic + '.csv'
        f[topic] = open(csv_file, 'w')
        f[topic].write('year, month, day, hour, min, second, microsecond, x, y, z, qw, qx, qy, qz\n')

    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=topics):
        if msg.header.stamp and msg.transform.translation and msg.transform.rotation and f[topic]:
            # Stamp
            f[topic].write(getStamp(msg.header.stamp))
            f[topic].write(",")
            f[topic].write(getTranslation(msg.transform.translation))
            f[topic].write(",")
            f[topic].write(getRotation(msg.transform.rotation))
            f[topic].write("\n")

    for topic in f:
        f[topic].close()
