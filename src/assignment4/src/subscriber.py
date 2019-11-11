#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo

def callback(data):
    rospy.loginfo('intrinsic parameters %s', data.K)
    rospy.loginfo('distortion coefficients %s', data.D)

def SpeedListener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('cameralistener', anonymous=False)
    rospy.Subscriber('/sensors/camera/infra1/camera_info', CameraInfo, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def main():
    SpeedListener()

print (main())
