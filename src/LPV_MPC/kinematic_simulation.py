#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
# from geometry_msgs.msg import Twist
from av_atras.msg import state
from function import Kinematic
from constants import *

# Create list to store the data
iteration=[]
car_pos_x = [] 
car_pos_y= [] 
car_psi = []
car_delta = []
car_x_dot = []
car_psi_dot = []
ref_pos_x = []
ref_pos_y = []
ref_psi = []
ref_x_dot = []
ref_psi_dot = []

def path_generator():
   # Plot the reference trajectory
    trajectory = 2
    t=np.arange(0,10+Tc,Tc) # duration of the entire manoeuvre
    lane_width=7 # [m]
    r=8
    f=0.01
    x_dot = 10
    
    # Define the x length, depends on the car's longitudinal velocity
    x=np.linspace(0,x_dot*t[-1],num=len(t))
    
    if trajectory==1:
        y=-9*np.ones(len(t))
    elif trajectory==2:
        y=9*np.tanh(t-t[-1]/2)
    elif trajectory==3:
        aaa=-28/100**2
        aaa=aaa/1.1
        if aaa<0:
            bbb=14
        else:
            bbb=-14
        y_1=aaa*(x+lane_width-100)**2+bbb
        y_2=2*r*np.sin(2*np.pi*f*x)
        # y=(y_1+y_2)/2
        y=(y_2)/2
    
    # Vector of x and y changes per sample time
    dx=x[1:len(x)]-x[0:len(x)-1]
    dy=y[1:len(y)]-y[0:len(y)-1]

    # Define the reference yaw angles
    psi=np.zeros(len(x))
    psiInt=psi
    psi[0]=np.arctan2(dy[0],dx[0])
    psi[1:len(psi)]=np.arctan2(dy[0:len(dy)],dx[0:len(dx)])

    # We want the yaw angle to keep track the amount of rotations
    dpsi=psi[1:len(psi)]-psi[0:len(psi)-1]
    psiInt[0]=psi[0]
    for i in range(1,len(psiInt)):
        if dpsi[i-1]<-np.pi:
            psiInt[i]=psiInt[i-1]+(dpsi[i-1]+2*np.pi)
        elif dpsi[i-1]>np.pi:
            psiInt[i]=psiInt[i-1]+(dpsi[i-1]-2*np.pi)
        else:
            psiInt[i]=psiInt[i-1]+dpsi[i-1]

    Vd = dx/Tc
    Wd = dpsi/Tc

    # Inisialisasi buffer dengan N elemen pertama dari data (N = horizon prediction)
    buffer_size = N
    Xr_dot = [Vd[i:i+buffer_size] for i in range(len(Vd) - buffer_size + 1)]
    Psi_dot = [Wd[i:i+buffer_size] for i in range(len(Wd) - buffer_size + 1)]
    Psi_N_array = [psiInt[i:i+buffer_size] for i in range(len(psiInt) - buffer_size + 1)]

    print(Psi_dot)
    print(Psi_N_array)
    # ====== PLOT TRAJECTORY =====
    # plt.plot(x,y,'b',linewidth=2,label='The trajectory')
    # # plt.plot(x,statesTotal[:,3],'--r',linewidth=2,label='Car position')
    # plt.xlabel('x-position [m]',fontsize=15)
    # plt.ylabel('y-position [m]',fontsize=15)
    # plt.grid(True)
    # plt.legend(loc='upper right',fontsize='small')
    # plt.ylim(-x[-1]/2,x[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
    # plt.show()
    return x,y,psiInt,Xr_dot,Psi_dot,Psi_N_array


Ac_pk, Bc = Kinematic.getModel()  # get model parameters
P, Ki, S= Kinematic.getMPCSet(Ac_pk,Bc) 
# set initial variabel



def openloop_control (data, psi_vector):
    
    # Construct Vector of Schedulling Variables
    pk_vector = np.zeros([3,N])
    pk_vector[0] = data.psi_dot_ref
    pk_vector[1] = data.x_dot_ref
    pk_vector[2] = psi_vector

    pk = pk_vector
    # pk = [data.psi_dot, data.x_dot_ref, data.psi]

    # pk_i = [pk_vector[0][0],pk_vector[1][0],pk_vector[2][1]] 
    # print("pk_vector : ", pk_vector[:,0])
    # print("pk", pk)
    # Construct the State-Space model
    X_k = np.array([[data.x], [data.y], [data.psi]])  # get current error state 
    U_k = np.array([[data.x_dot], [data.psi_dot]]) # get previous control signal

    # Construct reference signal for N horizon prediction
    # xr_dot_psi_e = [x * np.cos(data.psi) for x in data.x_dot_ref]
    xr_dot_psi_e = [data.x_dot_ref[i]*np.cos(psi_vector[i]) for i in range(len(psi_vector))]

    # print (data.x_dot_ref)
    # print (xr_dot_psi_e)
    
    Rc_k = np.array([[xr_dot_psi_e], [data.psi_dot_ref]])

    """
    In this work, the scheduling variables are states of the
    system whose desired values are known since the trajectory
    planner generates them. That is why we propose the use of
    such references as known scheduling variables for the entire
    optimization horizon being then the scheduling sequence
    Γ := [p(k), ..., p(k + N )]. In this way, we can compute the
    evolution of the model more accurately and in anticipation.

    NOTE  : HAL SUDAH DIPERTIMBANGKAN.
    """
    next_x_opt, u_opt = Kinematic.LPV_MPC(X_k, U_k, Rc_k, pk, Ac_pk, Bc, P, S)

    # ======= Calculate next state ========
    control_signal = u_opt
    # control_signal = U_k
    next_state = Kinematic.calculate_new_states(Ac_pk, pk[:,0], X_k, Bc, control_signal, Rc_k[:,:,0])

    print("=====================================")
    print("next_state : ", next_state)
    print("control_signal : ", control_signal)
    return next_state, control_signal


if __name__=='__main__':
    X_r, Y_r, Psi_r, xr_dot, psi_r_dot, Psi_N_array = path_generator()

    for i in range(len(xr_dot)):
        print  ("%d th loop" %i)
        car = state()
        if i == 0 :
            X_k = 0
            Y_k = 0 
            Psi_k = 0
            x_dot = 0
            psi_dot = 0
            delta_steer = 0
        else : 
            X_k = X_r[i] - next_state[0,0]
            Y_k = Y_r[i] - next_state[1,0]
            Psi_k = Psi_r[i] - next_state[2,0]
            x_dot = control_signal[0,0]
            psi_dot = control_signal[1,0]
            print("=======================")
            print("omega : ", control_signal[1,0])
            delta_steer= np.arctan(control_signal[1,0]*(lf+lr)/control_signal[0,0])
            print("delta : ", delta_steer)
            print("=======================")

        iteration.append(i)

        ref_pos_x.append(X_r[i])
        ref_pos_y.append(Y_r[i])
        ref_psi.append(Psi_r[i])
        ref_x_dot.append(xr_dot[i][0])
        ref_psi_dot.append(psi_r_dot[i][0])

        car_pos_x.append(X_k)
        car_pos_y.append(Y_k)
        car_psi.append(Psi_k)
        car_delta.append(delta_steer)

        car_x_dot.append(x_dot)
        car_psi_dot.append(psi_dot)

        # === kinematic control =====
        car.x = X_r[i] - X_k
        car.y = Y_r[i] - Y_k
        car.psi = Psi_r[i] - Psi_k
        # === kinematic reference =====
        car.psi_dot = psi_dot
        car.x_dot_ref = xr_dot[i]
        car.psi_dot_ref = psi_r_dot[i] 
        
        psi_error_vector = Psi_N_array[i]-Psi_k

        # print (Psi_N_array[i])
        # print ("Psi_k : ", Psi_k)
        # print (psi_error_vector)
        # print (car.psi)
        # print("===========================")
        # print("X_eror : ",X_r[i], " - ", X_k, " = ", car.x)
        # print("Y_error : ", Y_r[i], " - ", Y_k, " = ", car.y)
        # print("Psi_error : ",Psi_r[i], " - ", Psi_k, " = ", car.psi)
        # print("Vd  : ", car.x_dot_ref)
        # print("W omega : ", car.psi_dot_ref)
        # print("===========================")
    

        next_state, control_signal = openloop_control(car, psi_error_vector)

        # time.sleep(1)
        i = i+1

    print("Done")
    # print("Car Position (m)")
    # print("X: ", car_pos_x)
    # print("Y: ", car_pos_y)
    # print("Angle : ", car_psi)
    # print("Delta Steering : ", car_delta)

    # print("Dimensi X", len(car_pos_x))
    # print("Dimensi X_ref", len(ref_pos_x))
    # print("Dimensi Y", len(car_pos_y))
    # print("Dimensi Y_ref", len(ref_pos_y))
    # print("Dimensi Psi", len(car_psi))
    # print("Dimensi Psi_ref", len(ref_psi))

    print("Dimensi X_dot", len(car_x_dot))
    print("Dimensi X_dot_ref", len(xr_dot))
    print("Dimensi Psi_dot", len(car_psi_dot))
    print("Dimensi Psi_dot_ref", len(psi_r_dot))

    # ====== PLOT TRAJECTORY =====
    plt.plot(ref_pos_x,ref_pos_y,'b',linewidth=2,label='The trajectory')
    plt.plot(car_pos_x,car_pos_y,'--r',linewidth=2,label='Car position')
    plt.xlabel('x-position [m]',fontsize=15)
    plt.ylabel('y-position [m]',fontsize=15)
    plt.grid(True)
    plt.legend(loc='upper right',fontsize='small')
    plt.ylim(-ref_pos_x[-1]/2,ref_pos_x[-1]/2) # Scale roads (x & y sizes should be the same to get a realistic picture of the situation)
    plt.show()

    # Membuat subplot pertama
    plt.subplot(3, 1, 1)  # 3 baris, 1 kolom, subplot pertama
    plt.title('Longitudinal Velocity')
    plt.plot(iteration,ref_x_dot,'--r',linewidth=2,label='Setpoint')
    plt.plot(iteration,car_x_dot,'b',linewidth=2,label='Car')
    plt.xlabel('Iterasi')
    plt.ylabel('Vx')
    plt.legend(loc='upper right',fontsize='small')

    # Membuat subplot kedua
    plt.subplot(3, 1, 2)  # 3 baris, 1 kolom, subplot kedua
    plt.title('Angular Velocity (Rad/s)')
    plt.plot(iteration,ref_psi_dot,'--r',linewidth=2,label='Setpoint')
    plt.plot(iteration,car_psi_dot,'b',linewidth=2,label='Car')
    plt.xlabel('Iterasi')
    plt.ylabel('Omega')
    plt.legend(loc='upper right',fontsize='small')

    # Membuat subplot ketiga
    plt.subplot(3, 1, 3)  # 3 baris, 1 kolom, subplot ketiga
    plt.title('Steering Angle')
    plt.plot(iteration,car_delta,'b',linewidth=2,label='Car Steering')
    plt.xlabel('Iterasi')
    plt.ylabel('delta')
    plt.legend(loc='upper right',fontsize='small')

    # Menyesuaikan layout
    plt.tight_layout()

    # Menampilkan grafik
    plt.show()
