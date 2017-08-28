
The controller implements a simple integration algorithm which calculates the output position of the joint according to the given torque input. The relation between torque and position is defined by the second order equation:

	tau = J*X3 + b*X2 + k*X1

Where X1, X2 and X3 are, respectively, angular position, angular velocity and angular acceleration. J, b and k can be simply assumed as coeficients multiplying those variables, however, they would have a physical correspondece to Inertia, Dumping and Stiffness, however, since the process is discrete, we must find constants that will adjust J, b and k to the international standard. 

In order to achieve that, the system parameters are estimated from the system response with the MATLAB function _tfest_.