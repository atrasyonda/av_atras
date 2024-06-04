#!/usr/bin/env python

import rospy
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import Twist

def feedback_callback(data):
    # rospy.loginfo("I heard: %s", data.trajectories[data.selected_trajectory_idx].trajectory)
    rospy.loginfo("Success read data from /move_base/TebLocalPlannerROS/teb_feedback")
    trajectory = data.trajectories[data.selected_trajectory_idx].trajectory
    t = []
    v_x = []
    v_y = []
    omega = []
    x = []

    for point in trajectory:
      t.append(point.time_from_start.to_sec())
      v_x.append(point.velocity.linear.x)
      v_y.append(point.velocity.linear.y)
      omega.append(point.velocity.angular.z)
      x.append(point.pose.position.x)

    # print("Data", data.trajectories[data.selected_trajectory_idx].trajectory)
    # print ("Time-step", t)
    print ("Velocity X", v_x[0:10])
    print ("Velocity Y", v_y[0:10])
    print ("Omega", omega[0:10])
    # print ("X", x)
    # print ("Panjang data", len(x))
    # print ("Panjang data", len(v))

def feedback_cmd(data):
    rospy.loginfo("Success read data from /cmd_vel")
    print("Vx_cmd_vel : " ,data.linear.x)
    print("Omega_cmd_vel : " ,data.angular.z)
    print("===========================================")


def listener():
    # Initialize the node with the name 'listener'
    rospy.init_node('listener', anonymous=True)
    
    topic_name = "/move_base/TebLocalPlannerROS/teb_feedback" # define feedback topic here!
    rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) 
    rospy.Subscriber("/cmd_vel", Twist, feedback_cmd, queue_size = 1) 
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
