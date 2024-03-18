#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Читаем управление с клавиатуры!
---------------------------
Движение:
   w
a  s  d
   x

w/x : увеличить/уменьшить линейную скорость
a/d : увеличить/уменьшить угловую скорость
s : стоп

CTRL+C для выхода
"""

# Функция для получения символа с клавиатуры
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Основная функция узла
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('keyboard_driver')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    linear_speed = 0
    angular_speed = 0

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w' :
                linear_speed += 0.1
            elif key == 'x' :
                linear_speed -= 0.1
            elif key == 'a' :
                angular_speed += 0.1
            elif key == 'd' :
                angular_speed -= 0.1
            elif key == 's' :
                linear_speed = 0
                angular_speed = 0
            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
