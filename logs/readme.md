### Low Level Controller Tests


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