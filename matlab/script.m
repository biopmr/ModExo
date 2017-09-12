% Parameters
ms_to_s = 1/1000; % converts miliseconds to seconds
qc_to_rad = 2*pi/200000; % converts quadrature counts to radians
% 
% % Data structure
% dataEPOS = iddata(EPOSPosition*qc_to_rad,LoadCell,'Ts', 50);
% dataEPOS.OutputName  = '\Theta_{EPOS}';
% dataEPOS.OutputUnit  = 'rad';
% dataEPOS.InputName = 'tau_{Torque}';
% dataEPOS.InputUnit = 'mNm';
% dataEPOS.TimeUnit   = 'milliseconds';
% 
% dataArduino = iddata(X1*qc_to_rad*10000,LoadCell,'Ts', 50);
% dataArduino.OutputName  = '\Theta_{EPOS}';
% dataArduino.OutputUnit  = 'rad';
% dataArduino.InputName = 'tau_{Torque}';
% dataArduino.InputUnit = 'mNm';
% dataArduino.TimeUnit   = 'milliseconds';
% 
% % System Definition
% sysEPOSd = tfest(dataEPOS,2,0,nan,'Ts', 50); % EPOS discrete system
% sysEPOSc = d2c(sysEPOSd); % EPOS discrete system
% sysArduinod = tfest(dataArduino,2,0,nan,'Ts', 50); % Arduino discrete system
% sysArduinoc = d2c(sysArduinod); % EPOS discrete system
% 
% [Aed,Bed,Ced,Ded] = tf2ss(sysEPOSd.num,sysEPOSd.den) % EPOS discrete state spaces
% [Aad,Bad,Cad,Dad] = tf2ss(sysArduinod.num,sysArduinod.den) % Arduino discrete state spaces
% 
% [Aec,Bec,Cec,Dec] = tf2ss(sysEPOSc.num,sysEPOSc.den) % EPOS continuous state spaces
% [Aac,Bac,Cac,Dac] = tf2ss(sysArduinoc.num,sysArduinoc.den) % Arduino continuous state spaces

%% PLOT FOR MODEL REFERENCE TRACKING PERFORMANCE

[pks,locs] = findpeaks(LoadCell)
t0 = locs(1,1);

Colors = linspecer(6); 
figure('Units', 'pixels', ...
    'Position', [100 100 600 300]);
grid on
hold on

h1ModelReference = plot(Timems*ms_to_s, X1*qc_to_rad*10000);
h1SystemTracking = plot(Timems*ms_to_s, EPOSPosition*qc_to_rad);

set(h1ModelReference                       , ...
  'LineWidth'       ,    2        , ...
  'LineStyle'       , '-'      , ...
  'color'          , Colors(1,:)     );

set(h1SystemTracking                       , ...
 'LineWidth'       ,    2        , ...
 'LineStyle'       , '-'        , ...
 'color'          , Colors(2,:)     );

h1Legend = legend( ...
  [h1ModelReference, h1SystemTracking], ...
  '\theta_m', ...
  '\theta_e', ...
  'location', 'NorthEast' );

TrackingTitle  = title ('Model Reference Tracking');
h1YLabel = ylabel('Angle (rad)');
h1XLabel = xlabel('time (s)');

set([h1Legend, gca]             , ...
    'FontSize'   , 13           );

set( gca                       , ...
    'FontSize',         15     , ...
    'FontName'   , 'Helvetica' );

set(gca, ...
  'XMinorTick'  , 'on'      , ...
  'YMinorTick'  , 'on'      , ...
  'YGrid'       , 'off'      , ...
  'LineWidth'   , 1         );

set([h1XLabel, h1YLabel], ...
    'FontName'   , 'AvantGarde');

print -depsc2 modelReferenceTracking.eps
saveas(gcf,'modelReferenceTracking.png')