#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from controls import MotorHandler, GPIOHandler

SCALE_FACTOR = 50

class TeleopNode:
    def __init__(self):
        rospy.init_node('teleop_node', anonymous=True)
        self.motor_handler = MotorHandler([17], [27], GPIOHandler(mock_mode=True))

        self.subscriber = rospy.Subscriber("/turtle1/cmd_vel", Twist, self.cmd_vel_callback)


    def cmd_vel_callback(self, data):
        # Преобразование сообщения Twist в команды для MotorHandler
        linear_speed = data.linear.x * SCALE_FACTOR
        angular_speed = data.angular.z * SCALE_FACTOR

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
