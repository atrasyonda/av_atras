#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib as plt
import random
# from geometry_msgs.msg import Twist
from av_atras.msg import state
from geometry_msgs.msg import Twist
from function import Kinematic

Ac_pk, Bc = Kinematic.getModel()  # get model parameters
P, Ki, S= Kinematic.getMPCSet(Ac_pk,Bc) 

def callback(data,psi_vector):
    # Construct Vector of Schedulling Variables
    # pk = [data.psi_dot, data.x_dot_ref, data.psi]
    pk_vector = np.zeros([3,N])
    pk_vector[0] = data.psi_dot_ref
    pk_vector[1] = data.x_dot_ref
    pk_vector[2] = psi_vector

    pk = pk_vector

    # Construct the State-Space model
    X_k = np.array([[data.x], [data.y], [data.psi]])  # get current error state 
    U_k = np.array([[data.x_dot], [data.psi_dot]]) # get previous control signal

    print("current_state = ", X_k)
    print("current_control_signal = ", U_k)
     
    
    # Construct reference signal for N horizon prediction
    # xr_dot_psi_e = [x * np.cos(data.psi) for x in data.x_dot_ref]
    xr_dot_psi_e = [data.x_dot_ref[i]*np.cos(psi_vector[i]) for i in range(len(psi_vector))]

    Rc_k = np.array([[xr_dot_psi_e], [data.psi_dot_ref]])

    
    next_x_opt, u_opt = Kinematic.LPV_MPC(X_k, U_k, Rc_k, pk, Ac_pk, Bc, P, S)
    control_signal = u_opt[0]
    next_state = next_x_opt[1]
    print("===========================")
    # print("next_20_state : ", next_x_opt)
    # print("next_20_state : ", next_x_opt)
    print("next_state = ", next_state)
    print("control_signal = ", control_signal)
    print("===========================")

    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = control_signal[0]
    cmd_vel_msg.angular.z = control_signal[1]
    pub.publish(cmd_vel_msg)
    rospy.loginfo("Published x_dot: %f and psi_dot: %f to /cmd_vel", control_signal[0], control_signal[1])



if __name__=='__main__':
    rospy.init_node("lpv_mpc_node")
    rospy.loginfo("Node has been started")
    k_state = rospy.Subscriber("/car/state", state, callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10 )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()