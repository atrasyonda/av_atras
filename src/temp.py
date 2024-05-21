import numpy as np
Y_value = 1.0e-04* np.array([[0.0011,    0.0000,    0.0000],
                             [0.0000,    0.1371,   -0.0007],
                             [0.0000,   -0.0007,    0.0011]])

print(Y_value)

P = np.linalg.inv(Y_value)
print(P)

    
    