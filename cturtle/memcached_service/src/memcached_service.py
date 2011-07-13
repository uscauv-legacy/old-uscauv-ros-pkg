#!/usr/bin/env python
import roslib; roslib.load_manifest('memcached_service')
import rospy
from std_msgs.msg import String

SERVICE_NAME = 'memcached'
SERVICE_KEY = 'seabee3' # todo: implement msg w/ {key:value}

import memcache

def callback(data):
    try:
        mc = memcache.Client(['127.0.0.1:11211'], debug=0)
        if mc is not None:
            mc.set(SERVICE_KEY, data.data)
        else:
            rospy.loginfo("memcached client was None.")
    except Exception, e:
        rospy.loginfo("Exception caught:\n%s", e)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(SERVICE_NAME, String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
