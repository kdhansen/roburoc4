#!/usr/bin/env python
import roslib; roslib.load_manifest('viconToGps')
import rospy
import conversion as c
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

from geometry_msgs.msg import TransformStamped

latestViconMsg = TransformStamped()

latestViconMsg.transform.translation.x = 30

parentFrameLat = rospy.get_param('reference_latitude', 20)
parentFrameLong = rospy.get_param('reference_longitude', 30)
parentFrameAlt = rospy.get_param('reference_altitude', 40)
parentFrameAngle = rospy.get_param('Angle', 0)

def callback(viconData):
    ''' Saves the data when it gets a Vicon packet
    '''
    global latestViconMsg
    latestViconMsg = viconData
    #rospy.loginfo("Vicon position is %s.",[viconData.transform.translation.x, viconData.transform.translation.y, viconData.transform.translation.z])

def talker():
    # Subscribe to Vicon messages
    viconTopic = rospy.get_param('topic')
    rospy.Subscriber(viconTopic, TransformStamped, callback)

    # Start a publisher for the GPS messages
    pub = rospy.Publisher('GPS/position', NavSatFix) # FIXME

    # Start the node
    rospy.init_node('talker')

    
    # Populate the NavSatFix message from the parameter server
    statusMsg = NavSatStatus()
    statusMsg.status = rospy.get_param('status', -1)
    statusMsg.service = rospy.get_param('service', 1)

    fixMsg = NavSatFix()
    fixMsg.header.stamp = rospy.Time.now()
    fixMsg.header.frame_id = "/world"

    fixMsg.status = statusMsg
    fixMsg.position_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    #position could be modified with some gaussian noise added to it and then calculate the covariance matrix.
    fixMsg.position_covariance_type = rospy.get_param('position_covariance_type', 0); 

    while not rospy.is_shutdown():
        
        [fixMsg.longitude, fixMsg.latitude, fixMsg.altitude] = c.xyz2gps([parentFrameLong, parentFrameLat, parentFrameAlt], latestViconMsg.transform.translation.x, latestViconMsg.transform.translation.y, latestViconMsg.transform.translation.z, parentFrameAngle)

        statusMsg.status = rospy.get_param('status', statusMsg.status)
        statusMsg.service = rospy.get_param('service', statusMsg.service)
        # put the sigma and calculate the cov matrix here

        #rospy.loginfo([fixMsg.longitude, fixMsg.latitude, fixMsg.altitude]) 
        pub.publish(fixMsg)
        rospy.sleep(0.1)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
