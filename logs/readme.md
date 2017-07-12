### Loadcell Tests
The loadcell tests were made to evaluate the loadcell calibration defined by:
	    
	// LoadCell Calibration
	// [[loadcell_data_double = A*(force) + B]]
	float A = 0.0764; // slop steepness
	float B = 118712.7+5000; // offset
	float d = 0.105; // distance between the loadcell centers = 10,5cm
	    contactForce = (loadcell_data + B)*A;

#### 20170712_loadcell_tests_1
	1: 5000g
	2: 2*1068.7g
	3: 528.8g

#### 20170712_loadcell_tests_2
	1: 5000g
	2: 2*1068.7g
	3: 528.8g