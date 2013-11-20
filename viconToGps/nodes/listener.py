#!/usr/bin/env python
import roslib; roslib.load_manifest('viconToGps')
import rospy
from std_msgs.msg import String # this has to go
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

#baga subscriberu in talker si you're done with it, fara spin si alte prostii. :D

def callback(data):
    rospy.loginfo(rospy.get_name()+"Latitude is %s degrees",data.latitude)
	tudor = data.latitude

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("gpsFromVicon", NavSatFix, callback) #
    rospy.spin()

if __name__ == '__main__':
    listener()
