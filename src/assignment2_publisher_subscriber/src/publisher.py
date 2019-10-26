#!/usr/bin/env python
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand

PUBLISH_RATE = 10 #publish rate. The higher this is the high is the frequency of the publish
steeringCommand = NormalizedSteeringCommand()
speedCommand = SpeedCommand()

steeringCommand.value = 1.0
speedCommand.value = 0.3

def publish():
    rospy.init_node('speedPublisherNode')
    # publischer for steering to the wheels. Send messages with a small Queue.
    steeringPublisher = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size = 10)
    # publisher for publishing messages to speed topic (updating speed)
    speedPublisher = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size = 10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #turn the wheles
        steeringPublisher.publish(steeringCommand)
        #publish value for speed
        speedPublisher.publish(speedCommand)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
