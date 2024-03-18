#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(data):
    rospy.loginfo("Received cmd_vel message:")
    rospy.loginfo("Linear Components: [x: %s, y: %s, z: %s]" % (data.linear.x, data.linear.y, data.linear.z))
    rospy.loginfo("Angular Components: [x: %s, y: %s, z: %s]" % (data.angular.x, data.angular.y, data.angular.z))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/turtle1/cmd_vel", Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
