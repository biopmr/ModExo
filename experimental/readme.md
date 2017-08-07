### Low Level Controller Delay Tests
The joint response to a step input was tested for three different derivative gains. In these tests, the controller mode used was "EncoderControl", in which the reference position is given by manually turning the MA3 encoder. The first PID gains, in delay_test_1, were defined by the Regulation Tuning Wizard from EPOS Studio 2.00. We hypothesised that the delay would decrease as the Derivative Gain increased. So, for the delay_tests_2 and delay_tests_3, the derivative gains were simply increased to verify the change in the system response.

Below are listed the datasets and the respective Lower Level Controller (the EPOS) PID gains. This datasets are the CSV data generated as the output of the [ModExo code version](https://github.com/biopmr/ModExo/commit/fbe45a4ca06b29ec54b75cb432308c84095bc77f). 

#### 20170707-delay_tests_1.csv
	Position Regulator P-Gain = 2533
	Position Regulator I-Gain = 4772
	Position Regulator D-Gain = 6919
	Overshoot = 0
	Settling Time = 58.86-57.19 = 1.67 s
	Delay = 57.8-57.04 = 0.76 s

![Delay Test 1](https://biopmr.github.io/images/tests-delay_test_1-response.png)

#### 20170707-delay_tests_2.csv
	Position Regulator P-Gain = 2655
	Position Regulator I-Gain = 4948
	Position Regulator D-Gain = 15000
	Overshoot = 0
	Settling Time = 29.69-28.78 = 0.91 s
	Delay = 28.73-28.63 = 0.1 s

![Delay Test 2](https://biopmr.github.io/images/tests-delay_test_2-response.png)

#### 20170707-delay_tests_3.csv
	Position Regulator P-Gain = 2655
	Position Regulator I-Gain = 4948
	Position Regulator D-Gain = 30000
	Overshoot = 0
	Settling Time = 44.38-43.37 = 1.01 s
	Delay = 43.32-43.22 = 0.1 s

![Delay Test 3](https://biopmr.github.io/images/tests-delay_test_3-response.png)

#### Results
The graphs show two variables: the Theta Reference - in blue - corresponding to the signal of the MA3 Encoder, which gives the reference position for the low level controller and the Theta Tracking - in red - which corresponds to joint position given by the Joint Position Sensor.

The variable Delay is defined as the time gap between the instant of change in the theta reference and the instant of change in the theta tracking. As seen in the graphs, when the Derivative Gain is increased from 6919 to 15000 the Delay goes from 0.76 to 0.1 seconds, however, when increased from 15000 to 30000, no change is observed in the Delay.

The variable Settling Time is defined as the time gap between the instant when the theta reference reaches its final position and the instant when the theta tracking reaches its final position. Similarly to the Delay, when the Derivative Gain is increased from 6919 to 15000 the Settling Time goes from 1.67 to 0.91 seconds, however, when increased from 15000 to 30000, no significative change is observed in the Settling Time.

#### Discussion
Since the system dynamics will vary depending on the application, we are not concerned on finding an optimal relation between Derivative Gain and Delay: we are more interested in understanding the overall behaviour of the system.

The results indicate that an increase in the Derivative Gain will reduce the system response time up to a limit, which could be explained by a saturation of the actuators.

Further tests can be performed to observe the influence of the Integrative Gain, which should also result in a decrease of the Settling Time. Also, the step input can be made by software, to provide reptibility and other analysys possibilities. Issue #16 refers to this tests.

### Loadcell Tests
Static loadcell tests were made to evaluate the loadcell calibration defined below. The joint was positioned in the horizontal position, at \theta = \pi/2.  

	// LoadCell Calibration 
	// [[loadcell_data_double = A*(force) + B]]

	float A = 0.0764; // slop steepness
	float B = 118712.7+5000; // offset
	float d = 0.105; // distance between the loadcell centers = 10,5cm  

	contactForce = (loadcell_data + B)*A;

#### 20170712_loadcell_tests_1
	Weight 1: 5000g
	Weight 2: 2*1068.7g
	Weight 3: 528.8g

#### 20170712_loadcell_tests_2
	Weight 1: 5000g
	Weight 2: 2*1068.7g
	Weight 3: 528.8g

### Fall Tests
Two types of tests were made, in which the joint was released from the horizontal position at \theta = \pi/2 with a weight of 528.8g at the endpoint. In the first, the weight is left to fall freely, in the second, the weight fall is interrupted by a plateau. The tests were carried for two different impedances:

	i = 20
	b = 5
	k = 200

and

	i = 20
	b = 20
	k = 100
