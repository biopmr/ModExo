### Parameter Estimation

The controller implements a simple integration algorithm which calculates the output position of the joint according to the given torque input. The relation between torque and position is defined by the second order equation:

	tau = J*X3 + b*X2 + k*X1

Where X1, X2 and X3 are, respectively, angular position, angular velocity and angular acceleration. J, b and k can be simply assumed as coefficients multiplying those variables, although they could have a physical correspondence to Inertia, Dumping and Stiffness, since the process is discrete, they have no physical meaning. Below are described the steps to find constants that will adjust J, b and k to the international standard. 

#### Model Reference Tracking

As already mentioned in the control structure, the High Level Controller calculates the reference model position for a given input. The Low Level Controller task is to make the joint position as close as possible to that virtual position. In figures 1 and 2, it is possible to observe the tracking performance for the joint with no extra mass.

![Model Reference Tracking](https://biopmr.github.io/images/modelReferenceTracking.png)

![Model Reference Tracking 2](https://biopmr.github.io/images/modelReferenceTracking2.png)

The figures show two plots, \theta_m refers to the Model Reference Position calculated by the Arduino while \theta_e refers to the joint position given by the MILE Encoder in the motor. It is possible to observe that the curves are really close. Further experiments are required with higher impedance attached to the joint, however, for the analysis that will follow, we will consider that there is no significative difference in between the virtual dynamics requested by the Reference Model and the apparent dynamics performed by the Plant.


In order to achieve that, the system parameters are estimated from the system response with the MATLAB function _tfest_.

