#!/usr/bin/env python3
import rospy
import tf
from av_atras.msg import state
import matplotlib.pyplot as plt
from constants import *
from function import Kinematic

from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from teb_local_planner.msg import FeedbackMsg, TrajectoryPointMsg
from tf.transformations import euler_from_quaternion

class State_Publisher:
    def __init__(self):
        rospy.init_node('controller_state_publisher')

        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self.path_callback)
        # rospy.Subscriber("/move_base/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg, self.path_callback) 

        # Variables to store latest data
        self.latest_odom = None
        self.latest_plan = None
        self.tf_listener = tf.TransformListener()
        # self.latest_path_twist = None

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

        # print("Odom Frame", self.latest_odom.header.frame_id)
        self.odom_x = self.latest_odom.pose.pose.position.x
        self.odom_y = self.latest_odom.pose.pose.position.y

        # self.odom_psi = self.latest_odom.pose.pose.orientation.z

        # Extract orientation quaternion
        orientation_q = self.latest_odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Convert quaternion to euler angles
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.odom_psi = yaw # in radians

        # print("=============================")
        # print("z_odom", self.latest_odom.pose.pose.orientation.z)
        # print("yaw_quaternion", yaw)
        # print("=============================")

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

    def path_callback(self, message):
        self.latest_plan = message.trajectories[message.selected_trajectory_idx].trajectory
        # print("Path Frame", message.header.frame_id)
        self.path_x = []
        self.path_y = []
        self.path_psi = []
        self.path_x_dot = []
        self.path_psi_dot = []
        path_yaw = []

        try:
            self.tf_listener.waitForTransform('/odom', message.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/odom', message.header.frame_id, rospy.Time(0))
            rotation_matrix = tf.transformations.quaternion_matrix(rot)
            
            for data in self.latest_plan:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = message.header.frame_id
                pose_stamped.header.stamp = rospy.Time(0)  # Use latest available transform
                pose_stamped.pose.position = data.pose.position
                pose_stamped.pose.orientation = data.pose.orientation

                twist_stamped = TwistStamped()
                twist_stamped.header.frame_id = message.header.frame_id
                twist_stamped.header.stamp = rospy.Time(0)  # Use latest available transform
                twist_stamped.twist.linear = data.velocity.linear
                twist_stamped.twist.angular = data.velocity.angular

                transformed_pose = self.tf_listener.transformPose('/odom', pose_stamped)
                self.path_x.append(transformed_pose.pose.position.x)
                self.path_y.append(transformed_pose.pose.position.y)
                # Extract yaw from quaternion
                orientation_q = transformed_pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
                self.path_psi.append(yaw)

                # self.path_psi.append(transformed_pose.pose.orientation.z)
                # path_yaw.append(yaw)


                linear_velocity = data.velocity.linear
                transformed_linear_velocity = rotation_matrix.dot([linear_velocity.x, linear_velocity.y, linear_velocity.z, 0.0])[:3]
                # self.path_x_dot.append(transformed_linear_velocity[0])
                self.path_x_dot.append(data.velocity.linear.x)
                
                # Transform angular velocity
                angular_velocity = data.velocity.angular
                transformed_angular_velocity = rotation_matrix.dot([angular_velocity.x, angular_velocity.y, angular_velocity.z, 0.0])[:3]
                # self.path_psi_dot.append(transformed_angular_velocity[2])
                self.path_psi_dot.append(data.velocity.angular.z)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform failed: %s", str(e))
    
        # print("Panjang data path X", len(self.path_x))
        # print("Panjang data path Psi", len(self.path_psi))
        # print("Panjang reference VC", len(self.path_x_dot))
        # print("Panjang reference Psidot", len(self.path_psi_dot))

        # for data in self.latest_plan:
        #     self.path_x.append(data.pose.position.x)
        #     self.path_y.append(data.pose.position.y)
        #     self.path_psi.append(data.pose.orientation.z)

        #     orientation_q = data.pose.orientation
        #     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        #     # Convert quaternion to euler angles
        #     (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        #     # self.odom_psi = yaw # in radians


        #     path_yaw.append(yaw)

        #     self.path_x_dot.append(data.velocity.linear.x)
        #     self.path_psi_dot.append(data.velocity.angular.z)



        # print("=============================")
        # print("z_path", self.path_psi)
        # print("=============================")
        # print("yaw_quaternion", path_yaw)

        # print("=============================")
        # print("Path_x : ", self.path_x)
        # print("=============================")
        # print("Path_y : ", self.path_y)
        # print("=============================")
        # print("Path_psi : ", self.path_psi)
        # print("=============================")
        # print("Path_vx : ", self.path_x_dot)
        # print("=============================")
        # print("Path_omega : ", self.path_psi_dot)
        # print("=============================")
        # print("Length : ", len(self.path_x))
        return self.path_x, self.path_y, self.path_psi, self.path_x_dot, self.path_psi_dot, self.latest_plan

    # def cmd_vel_callback(self, data):
    #     self.latest_path_twist = data
    #     self.path_x_dot = data.linear.x
    #     self.path_psi_dot = data.angular.z
    #     # print("=============================")
    #     # print("Path_x_dot : ", self.path_x_dot)
    #     # print("=============================")
    #     # print("Path_psi_dot : ", self.path_psi_dot)
    #     return self.path_x_dot, self.path_psi_dot

    def collect_car_state(self):
        position_error = None
        velocity_state = None
        velocity_reference = None
        if self.latest_plan is not None and self.latest_odom is not None:
            position_error = np.zeros([3,len(self.path_x)])
            velocity_state = []
            velocity_reference = np.zeros([2,len(self.path_x_dot)])

            X_error = [self.path_x[i]- self.odom_x for i in range(len(self.path_x))] 
            Y_error = [self.path_y[i] - self.odom_y for i in range(len(self.path_y))] 
            Psi_error = [self.path_psi[i] - self.odom_psi for i in range(len(self.path_psi))]

            Vx = self.odom_x_dot
            Vy = self.odom_y_dot
            Psi_dot = self.odom_psi_dot

            position_error[0] = X_error
            position_error[1] = Y_error
            position_error[2] = Psi_error
            velocity_state = [Vx, Vy, Psi_dot]

            # Vx_ref = [np.sqrt((self.path_x[i+1]-self.path_x[i])**2 + (self.path_y[i+1]-self.path_y[i])**2)/0.1 for i in range(len(self.path_x)-1)]
            # Omega_ref = [(self.path_psi[i+1]-self.path_psi[i])/0.1 for i in range(len(self.path_psi)-1)]

            # X_dot_ref = [self.path_x_dot*np.cos(Psi_error[i]) for i in range(len(self.path_psi))]
            # Psi_dot_ref = [self.path_psi_dot for _ in range(len(X_error))]

            velocity_reference[0, 0:len(self.path_x_dot)] = self.path_x_dot
            velocity_reference[1, 0:len(self.path_psi_dot)] = self.path_psi_dot
            
            # velocity_reference[0,1:] = Vx_ref
            # velocity_reference[1,1:] = Omega_ref 

            # print("============================")
            # print("Vx_reference", velocity_reference[0])
            # print("Omega_reference", velocity_reference[1])
            # print("X_error", position_error[0])
            # print("Y_error", position_error[1])
            # print("Psi_error", position_error[2])
            # print("============================")
        else:
            rospy.logwarn("Either Path/Odom/Cmd_vel Message is Missing, check /move_base local plan,/odom and /move_base/cmd_vel")
            """
            NOTE : Velocity ref dari cmd_vel cuma 1 step, yang saya ulang 20 kali sbg referensi
            sepanjang horizon. Sepertinya ada yg salah karena seharusnya 20 step kedepan nilainya
            beda-beda. kalau dari simulasi kemarin xdot= (x2-x1)/T begitu juga psidot= (psi2-psi1)/T
            """
        return position_error, velocity_state, velocity_reference

    def run(self):
        pub = rospy.Publisher("/car/state", state, queue_size=10 )
        rate = rospy.Rate(10)  # 10 Hz
        car_msg = state()
        while not rospy.is_shutdown():
            position_error, velocity_state, velocity_reference = self.collect_car_state()

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