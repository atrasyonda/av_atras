#!/usr/bin/env python3
import numpy as np

n = 3  # number of states
m = 2  # number of inputs
N = 20 # horizon period

Tc=0.1      # time sampling for kinematic control 
Td=0.01     # time sampling for dynamic control 

# ==================  CAR MODEL ========================= #
# ==========  LPV-MPC for Autonomous Vehicle ============ #


# ==================  CAR MODEL ========================= #
# ==========  LPV-MPC for Autonomous Vehicle ============ #

# car constants
mass = 683       # massa mobil
lf = 0.758    # panjang antara CoM dg roda depan (m)
lr = 1.036    # panjang antara CoM dg roda belakang (m)
I = 560.94      # momen inersia (kg.m^2)
Caf = 24000 # koefisien kekakuan roda depan (N/rad)
Car = 21000 # koefisien kekakuan roda belakang (N/rad)
mass = 683       # massa mobil
lf = 0.758    # panjang antara CoM dg roda depan (m)
lr = 1.036    # panjang antara CoM dg roda belakang (m)
I = 560.94      # momen inersia (kg.m^2)
Caf = 24000 # koefisien kekakuan roda depan (N/rad)
Car = 21000 # koefisien kekakuan roda belakang (N/rad)

rho = 1.184 # densitas udara (kg/m^3)
Cd = 0.36     # koefisien drag
miu = 1   # koefisien gesekan
Af = 1.91     # front sectional area of the vehicle (m^2)
g = 9.8     # konstanta gravitasi (m/s^2)
Cd = 0.36     # koefisien drag
miu = 1   # koefisien gesekan
Af = 1.91     # front sectional area of the vehicle (m^2)
g = 9.8     # konstanta gravitasi (m/s^2)

# schedulling vector variable
psi_dot_min= -1.42  # batas bawah kecepatan anguler (rad/s)
psi_dot_max= 1.42   # batas atas kecepatan anguler (rad/s)
xr_dot_min= 0.1     # batas bawah reference kecepatan linier (m/s)
xr_dot_max= 20      # batas atas reference kecepatan linier (m/s)
psi_min= -0.05      # batas bawah sudut orientasi (rad)
psi_max= 0.05       # batas atas sudut orientasi (rad)

delta_min = -0.8  # -85 deg batas bawah sudut steering (rad) --> referensi -0.25 (-15 deg)
delta_max = 0.8   # 1.5  85 deg batas atas sudut steering (rad) --> referensi 0.25 (15 deg)
x_dot_min = 0.1     # batas bawah kecepatan longitudinal / linier (m/s) --> referensi 0.1
x_dot_max = 10      # batas atas kecepatan longitudinal / linier (m/s) --> referensi 20
y_dot_min = -1      # batas bawah kecepatan lateral (m/s)
y_dot_max = 1       # batas atas kecepatan lateral (m/s)

a_min = -2         # batas bawah percepatan (m/s^2)
a_max = 2          # batas atas percepatan (m/s^2)


# Kinematic LPV-MPC Controller Design
u_max = np.array([[x_dot_max], [psi_dot_max]])  # batas atas input (X_dot dan Psi_dot) --> jurnal 
u_min = np.array([[x_dot_min], [psi_dot_min]])  # batas bawah input (X_dot dan Psi_dot)--> jurnal

delta_u_max = np.array([[a_max], [0.3]])  # batas atas perubahan input --> jurnal
delta_u_min = np.array([[a_min], [-0.3]])  # batas bawah perubahan input --> jurnal

Q_k =  0.9*np.diag([0.33, 0.33, 0.33]) # kinematic MPC state weight matrix --> JURNAL
R_k =  0.1*np.diag([0.8, 0.2])# kinematic MPC input weight matrix --> JURNAL

Q_ts = np.diag([1,1,3]) # LMI Kinematic State Weight Matrix --> JURNAL
R_ts = np.diag([1,3])  # LMI Kinematic Input Weight Matrix --> JURNAL

# Dynamic LPV-LQR Controller Design
Q_d = 0.9*np.diag([0.66, 0.01, 0.33])  # LMI Dynamic State Weight Matrix --> JURNAL
R_d = 0.1*np.diag([0.5, 0.5])  # LMI Dynamic Input Weight Matrix --> JURNAL

# inisiasi state
# psi_r_dot = 0
# xr_dot = 0
# psi = 0

# x_dot = 0
# y_dot = 0
# delta = 0
