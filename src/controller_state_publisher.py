#!/usr/bin/env python3
import rospy
import random
from autonomous_vehicle.msg import state
import matplotlib.pyplot as plt
from constants import *
from function import Kinematic

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class State_Publisher:
    def __init__(self):
        rospy.init_node('controller_state_publisher')

        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.path_callback)
        rospy.Subscriber("/move_base/cmd_vel", Twist, self.cmd_vel_callback)

        # Variables to store latest data
        self.latest_odom = None
        self.latest_path_pose = None
        self.latest_path_twist = None

        self.path_x = None
        self.path_y = None
        self.path_psi = None
        self.path_x_dot = None
        self.path_psi_dot = None

        self.odom_x = None
        self.odom_y = None
        self.odom_psi = None
        self.odom_x_dot = None
        self.odom_y_dot = None
        self.odom_psi_dot = None

    def odom_callback(self, data):
        self.latest_odom = data
        # print(data)
        self.odom_x = self.latest_odom.pose.pose.position.x
        self.odom_y = self.latest_odom.pose.pose.position.y
        self.odom_psi = self.latest_odom.pose.pose.orientation.z

        self.odom_x_dot = self.latest_odom.twist.twist.linear.x
        self.odom_y_dot = self.latest_odom.twist.twist.linear.y
        self.odom_psi_dot = self.latest_odom.twist.twist.angular.z
        # print("=============================")
        # print("Odom_x : ", self.odom_x)
        # print("=============================")
        # print("Odom_y : ", self.odom_y)
        # print("=============================")
        # print("Odom_psi : ", self.odom_psi)
        # print("=============================")
        # print("Odom_x_dot : ", self.odom_x_dot)
        # print("=============================")
        # print("Odom_y_dot : ", self.odom_y_dot)
        # print("=============================")
        # print("Odom_psi_dot : ", self.odom_psi_dot)
        # print("=============================")
        return self.odom_x, self.odom_y, self.odom_psi, self.odom_x_dot, self.odom_y_dot, self.odom_psi_dot, self.latest_odom

    def path_callback(self, data):
        self.latest_path_pose = data.poses
        self.path_x = []
        self.path_y = []
        self.path_psi = []
        for pose_stamped in self.latest_path_pose:
            pose = pose_stamped.pose
            position = pose.position
            orientation = pose.orientation

            # Extract x, y, and z coordinates
            x = position.x
            y = position.y
            z = orientation.z  # Assuming orientation z represents the z component of quaternion
            
            self.path_x.append(x)
            self.path_y.append(y)
            self.path_psi.append(z)
        # print("=============================")
        # print("Path_x : ", self.path_x)
        # print("=============================")
        # print("Path_y : ", self.path_y)
        # print("=============================")
        # print("Path_psi : ", self.path_psi)
        # print("=============================")
        # print("Length : ", len(self.path_x))
        return self.path_x, self.path_y, self.path_psi, self.latest_path_pose

    def cmd_vel_callback(self, data):
        self.latest_path_twist = data
        self.path_x_dot = data.linear.x
        self.path_psi_dot = data.angular.z
        # print("=============================")
        # print("Path_x_dot : ", self.path_x_dot)
        # print("=============================")
        # print("Path_psi_dot : ", self.path_psi_dot)
        
        return self.path_x_dot, self.path_psi_dot

    def collect_car_state(self):
        position_error = None
        velocity_state = None
        velocity_reference = None
        if self.latest_path_pose is not None and self.latest_odom is not None:
            X_error = self.path_x[0] - self.odom_x
            Y_error = self.path_y[0] - self.odom_y
            Psi_error = self.path_psi[0] - self.odom_psi
            Vx = self.odom_x_dot
            Vy = self.odom_y_dot
            Psi_dot = self.odom_psi_dot
            position_error = [X_error,Y_error,Psi_error]
            velocity_state = [Vx, Vy, Psi_dot]
        else:
            rospy.logwarn("Path Message is Missing, check /move_base local plan")
            rospy.logwarn("Odom Message is Missing, check /odom")
            # return None

        if self.latest_path_twist is not None:
            X_dot_ref = [self.path_x_dot for _ in range(20)]
            Psi_dot_ref = [self.path_psi_dot for _ in range(20)]
            velocity_reference = np.zeros([2,20])
            velocity_reference[0, :] = X_dot_ref
            velocity_reference[1, :] = Psi_dot_ref 
            """
            NOTE : Velocity ref dari cmd_vel cuma 1 step, yang saya ulang 20 kali sbg referensi
            sepanjang horizon. Sepertinya ada yg salah karena seharusnya 20 step kedepan nilainya
            beda-beda. kalau dari simulasi kemarin xdot= (x2-x1)/T begitu juga psidot= (psi2-psi1)/T
            """

        else:
            rospy.logwarn("Velocity reference Message is Missing, check /move_base/cmd_vel ")
            # return None
        
        return position_error, velocity_state, velocity_reference

    def run(self):
        pub = rospy.Publisher("/car/state", state, queue_size=10 )
        rate = rospy.Rate(10)  # 10 Hz
        car_msg = state()
        while not rospy.is_shutdown():
            position_error, velocity_state, velocity_reference = self.collect_car_state()
            # velocity_error = self.calculate_velocity_error()

            if position_error is not None:
                rospy.loginfo("Position Error: %s", position_error)
                car_msg.x = position_error[0]
                car_msg.y = position_error[1]
                car_msg.psi = position_error[2]

            if velocity_state is not None:
                rospy.loginfo("Velocity State: %s", velocity_state)
                car_msg.x_dot = velocity_state[0]
                car_msg.y_dot = velocity_state[1]
                car_msg.psi_dot = velocity_state[2]

            if velocity_reference is not None:
                rospy.loginfo("Velocity Reference: %s", velocity_reference)
                car_msg.x_dot_ref = velocity_reference[0]
                car_msg.psi_dot_ref = velocity_reference[1]

            pub.publish(car_msg)

            rate.sleep()

if __name__ == '__main__':
    state_publisher_node = State_Publisher()
    state_publisher_node.run()