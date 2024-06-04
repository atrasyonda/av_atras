#!/usr/bin/env python3
import numpy as np
from function import Kinematic
Ac_pk, Bc = Kinematic.getModel()  # get model parameters
# P, Ki, S= Kinematic.getMPCSet(Ac_pk,Bc) 

P = np.array([
    [6.7619e+07, -1.0e+02, -5.0e+02],
    [-1.0e+02, 2.4e+07, 1.1e+06],
    [-5.0e+02, 1.1e+06, 4.757e+07]
])

# Define matrix S
S = np.array([
    [2.816e+08, -1.0e+02, 2.0e+05],
    [-1.0e+02, 2.863e+08, 1.149e+08],
    [2.0e+05, 1.149e+08, 2.5493e+09]
])

print("Ac_pk : ", Ac_pk)
print("Bc : ", Bc)
print("P : ", P)
print("S",S)

eigenvalues = np.linalg.eigvals(P)
if np.all(eigenvalues >= 0):
    print("P adalah matriks positif semidefinit.")
else:
    print("P bukan matriks positif semidefinit.")

eigenvalues = np.linalg.eigvals(S)
if np.all(eigenvalues >= 0):
    print("S adalah matriks positif semidefinit.")
else:
    print("S bukan matriks positif semidefinit.")