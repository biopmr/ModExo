
%% VARIABLE EDITING
ms_to_s = 1/1000; % converts miliseconds to seconds
qc_to_rad = 2*pi/200000; % converts quadrature counts to radians

data = iddata(EPOSPosition*qc_to_rad,LoadCell,'Ts', 50);
data.OutputName  = '\Theta_{EPOS}';
data.OutputUnit  = 'rad';
data.InputName = 'tau_{Torque}';
data.InputUnit = 'mNm';
data.TimeUnit   = 'milliseconds';
plot(data)

sysTF = tfest(data,2,0,nan,'Ts', 50)
sys = tf(sysTF.num, sysTF.den,0.05)

sysSS = tf2ss(sysTF.num,sysTF.den)

[A,B,C,D] = tf2ss(sysTF.num,sysTF.den)
sysTFC = d2c(sysTF)


% >> sysTF = tfest(data,2,0,nan)
% 
% sysTF =
% 
%  From input "tau_{Torque}" to output "\Theta_{EPOS}":
%                          0.001975
%  exp(-100*s) * ----------------------------
%                s^2 + 0.001844 s + 3.959e-06
% 
%Continuous-time identified transfer function.
%
%Parameterization:
%   Number of poles: 2   Number of zeros: 0
%   Number of free coefficients: 3
%   Use "tfdata", "getpvec", "getcov" for parameters and their uncertainties.

%Status:                                          
%Estimated using TFEST on time domain data "data".
%Fit to estimation data: 98.25% (simulation focus)
%FPE: 4.068e+04, MSE: 3.91e+04 