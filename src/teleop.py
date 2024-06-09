#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
             
w/, : increase/decrease linear velocity
j/l : increase/decrease steering angle
(space) : stop

CTRL-C to quit
"""

move_bindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    ' ': (0, 0)
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(key)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('robot_teleop_key')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    linear_vel = 0.0
    steering_angle = 0.0
    linear_vel_step = 0.1
    steering_angle_step = 0.1

    try:
        print(msg)
        while(1):
            key = get_key()
            if key in move_bindings.keys():
                linear_vel += move_bindings[key][0] * linear_vel_step
                steering_angle += move_bindings[key][1] * steering_angle_step
            elif key == '\x03':
                break
            elif key == 'x':
                linear_vel = 0.0
                steering_angle = 0.0
            else:
                linear_vel = 0.0
                steering_angle = 0.0

            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = steering_angle
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
