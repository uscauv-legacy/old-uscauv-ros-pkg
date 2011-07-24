#!/usr/bin/env python
import roslib; roslib.load_manifest('memcached_service')
import rospy
import random
from std_msgs.msg import String
from django.utils import simplejson as json

def get_rand_value():
    return random.randint(0, 360)

def randomly_fill_data(data):
    for k,v in data.iteritems():
        data[k] = str(get_rand_value())
    return data

def talker():
    pub = rospy.Publisher('memcached', String)
    rospy.init_node('talker')
    seabee_data = { 'ext_pressure' : 'n/a',
                    'int_pressure' : 'n/a',
                    'heading' : 'n/a' }
    while not rospy.is_shutdown():
        seabee_data = randomly_fill_data(seabee_data)
        data_str = json.dumps(seabee_data)
        rospy.loginfo(data_str)
        pub.publish(String(data_str))
        rospy.sleep(0.5)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
