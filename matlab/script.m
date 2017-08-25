% Parameters
ms_to_s = 1/1000; % converts miliseconds to seconds
qc_to_rad = 2*pi/200000; % converts quadrature counts to radians

% Data structure
dataEPOS = iddata(EPOSPosition*qc_to_rad,LoadCell,'Ts', 50);
dataEPOS.OutputName  = '\Theta_{EPOS}';
dataEPOS.OutputUnit  = 'rad';
dataEPOS.InputName = 'tau_{Torque}';
dataEPOS.InputUnit = 'mNm';
dataEPOS.TimeUnit   = 'milliseconds';

dataArduino = iddata(X1*qc_to_rad*10000,LoadCell,'Ts', 50);
dataArduino.OutputName  = '\Theta_{EPOS}';
dataArduino.OutputUnit  = 'rad';
dataArduino.InputName = 'tau_{Torque}';
dataArduino.InputUnit = 'mNm';
dataArduino.TimeUnit   = 'milliseconds';

% System Definition
sysEPOSd = tfest(dataEPOS,2,0,nan,'Ts', 50); % EPOS discrete system
sysEPOSc = d2c(sysEPOSd); % EPOS discrete system
sysArduinod = tfest(dataArduino,2,0,nan,'Ts', 50); % Arduino discrete system
sysArduinoc = d2c(sysArduinod); % EPOS discrete system

[Aed,Bed,Ced,Ded] = tf2ss(sysEPOSd.num,sysEPOSd.den) % EPOS discrete state spaces
[Aad,Bad,Cad,Dad] = tf2ss(sysArduinod.num,sysArduinod.den) % Arduino discrete state spaces

[Aec,Bec,Cec,Dec] = tf2ss(sysEPOSc.num,sysEPOSc.den) % EPOS continuous state spaces
[Aac,Bac,Cac,Dac] = tf2ss(sysArduinoc.num,sysArduinoc.den) % Arduino continuous state spaces

