% Parameters
ms_to_s = 1/1000; % converts miliseconds to seconds
qc_to_rad = 2*pi/200000; % converts quadrature counts to radians
 
% Data structure
b1k2T100Data = iddata(X1*qc_to_rad*10000,LoadCell/1000,'Ts', 0.050);
b1k2T100Data.OutputName  = '\Theta_{EPOS}';
b1k2T100Data.OutputUnit  = 'rad';
b1k2T100Data.InputName = 'tau_{Torque}';
b1k2T100Data.InputUnit = 'Nm';
b1k2T100Data.TimeUnit   = 'seconds';

b1k2T100THETA = X1*qc_to_rad*10000;
b1k2T100LoadCell = LoadCell/1000;
b1k2T100Times = linspace(0,0.05*(length(X1)-1),length(X1))';

% System Definition
sysArduinod = tfest(b1k2T100Data,2,0,nan,'Ts', 50); % Arduino discrete system
sysArduinoc = d2c(sysArduinod); % Arduino continuous system

[Aad,Bad,Cad,Dad] = tf2ss(sysArduinod.num,sysArduinod.den) % Arduino discrete state spaces
[Aac,Bac,Cac,Dac] = tf2ss(sysArduinoc.num,sysArduinoc.den) % Arduino continuous state spaces

sysArduino2 = ss(Aac, Bac, [0 Cac(1,2)], 0); % Forces the state spaces representation to a known format

estimatedRatio = (Aac(1,1)/Aac(1,2));

% PLOT

[pks,locs] = findpeaks(LoadCell)
% t0 = locs(1,1);

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

TrackingTitle  = title ('Decreasing Step');
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

print -depsc2 decreasingStep.eps
saveas(gcf,'decreasingStep.png')

