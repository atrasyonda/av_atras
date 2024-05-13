# This is the message that will send to controller, obtained from many topics
#=== FROM ODOMETRY ESTIMATION ===#
float64 x		# error position x --> x_ref - x_est
float64 y		# error position y --> y_ref - y_est
float64 psi		# error orientation z --> psi_ref - psi_est

float64 x_dot		# velocity x 
float64 y_dot		# velocity y
float64 psi_dot	# angular velocity (omega)

#=== FROM REFERENCE TRAJECTORY ===#
float64[] x_dot_ref 	# Vx_reference --> berupa list berisi rk, rk+1 sampai rk+N-1 (N size)
float64[] psi_dot_ref	# omega_reference --> berupa list berisi rk, rk+1 sampai rk+N-1 (N size)

#=== FROM OUTPUT CONTROLLER (FEEDBACK) ===#
float64 delta
