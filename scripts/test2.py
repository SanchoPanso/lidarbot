#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class SmoothTurtleControl:
    def __init__(self):
        rospy.init_node('smooth_turtle_control')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_velocity = Twist()

    def smooth_velocity(self, target_velocity, acceleration):
        while not rospy.is_shutdown():
            # Плавное изменение скорости
            if self.current_velocity.linear.x < target_velocity.linear.x:
                self.current_velocity.linear.x += acceleration
            elif self.current_velocity.linear.x > target_velocity.linear.x:
                self.current_velocity.linear.x -= acceleration

            # Здесь можно добавить аналогичный код для угловой скорости

            self.pub.publish(self.current_velocity)
            self.rate.sleep()

if __name__ == '__main__':
    control = SmoothTurtleControl()
    target = Twist()  # Задайте желаемую скорость здесь
    acceleration = 0.1  # Задайте желаемое ускорение здесь
    control.smooth_velocity(target, acceleration)
