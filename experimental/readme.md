### Low Level Controller Delay Tests
The joint response to a step input was tested for three different derivative gains. The first PID gains, in delay_test_1, were defined by the Regulation Tuning Wizard from EPOS Studio 2.00. We hypothesised that the delay would decrease as the Derivative Gain increased. So, for the delay_tests_2 and delay_tests_3, the derivative gains were simply increased to verify the change in the system response.

Below are listed the datasets and the respective Lower Level Controller (the EPOS) PID gains. This datasets are the CSV data generated as the output of the [ModExo code version](https://github.com/biopmr/ModExo/commit/fbe45a4ca06b29ec54b75cb432308c84095bc77f). 

#### 20170707-delay_tests_1.csv
	Position Regulator P-Gain = 2533
	Position Regulator I-Gain = 4772
	Position Regulator D-Gain = 6919
	Overshoot = 0
	Settling Time = 58.86(57.19)-57.04
	Delay = 57.8-57.04

![Delay Test 1](https://biopmr.github.io/images/tests-delay_test_1-response.png)

#### 20170707-delay_tests_2.csv
	Position Regulator P-Gain = 2655
	Position Regulator I-Gain = 4948
	Position Regulator D-Gain = 15000
	Overshoot = 0
	Settling Time = 28.63(28.78)-29.69
	Delay = 28.63-28.73

![Delay Test 2](https://biopmr.github.io/images/tests-delay_test_2-response.png)

#### 20170707-delay_tests_3.csv
	Position Regulator P-Gain = 2655
	Position Regulator I-Gain = 4948
	Position Regulator D-Gain = 30000
	Overshoot = 0
	Settling Time = 43.22(43.37)-44.38
	Delay = 43.22-43.32

![Delay Test 3](https://biopmr.github.io/images/tests-delay_test_3-response.png)

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
