#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from controls import MotorHandler, GPIOHandler

SCALE_FACTOR = 50

class TeleopNode:
    def __init__(self):
        rospy.init_node('teleop_node', anonymous=True)
        self.motor_handler = MotorHandler([17], [27], GPIOHandler(mock_mode=True))

        self.subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)


    def cmd_vel_callback(self, data):
        # Преобразование сообщения Twist в команды для MotorHandler
        linear_speed = data.linear.x
        angular_speed = data.angular.z
        
        print("Received cmd_vel message:")
        print("Linear Components: [x: %s, y: %s, z: %s]" % (data.linear.x, data.linear.y, data.linear.z))
        print("Angular Components: [x: %s, y: %s, z: %s]" % (data.angular.x, data.angular.y, data.angular.z))

        # Примерная логика управления
        if linear_speed > 0:
            self.motor_handler.move_forward(abs(linear_speed))
        elif linear_speed < 0:
            self.motor_handler.move_backward(abs(linear_speed))
        elif angular_speed > 0:
            self.motor_handler.turn_right(abs(angular_speed))
        elif angular_speed < 0:
            self.motor_handler.turn_left(abs(angular_speed))
        else:
            self.motor_handler.stop()

if __name__ == '__main__':
    node = TeleopNode()
    rospy.spin()
