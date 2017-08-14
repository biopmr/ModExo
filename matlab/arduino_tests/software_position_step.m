Colors = linspecer(6); 

%% VARIABLE EDITING
ms_to_s = 1/1000; % converts miliseconds to seconds
qc_to_rad = 2*pi/200000; % converts quadrature counts to radians
l = 0.105; % length of the torque arm in cm
g = 9.81; % gravity

% use the code below delete unnecessary lines
% initial=1;
% final=2000;
%
%  for i = 1:(final-initial+1)
%    Encoder(initial,:) = [];
%    EPOSCurrent(initial,:) = [];
%    EPOSPosition(initial,:) = [];
%    LoadCell(initial,:) = [];
%    SampleNumber10000samplespersecond(initial,:) = [];
%    Timems(initial,:) = [];
%    X1(initial,:) = [];
%    X2(initial,:) = [];
%    X3(initial,:) = [];
%  end

%% ERROR CALCULATIONS
% mean_error_5kg = ((Torque(1:306) - LoadCell(1:306)).^2).^0.5;
% mean_error_500g = ((Torque(600:1100) - LoadCell(600:1100)).^2).^0.5;
% mean_error_2kg = ((Torque(1515:1699) - LoadCell(1515:1699)).^2).^0.5;
% 
% percentage_error_5kg = mean_error_5kg./Torque(1:306);
% mean_percentage_error_5kg = sum(percentage_error_5kg)/length(mean_error_5kg)
% 
% percentage_error_500g = mean_error_500g./Torque(600:1100);
% mean_percentage_error_500g = sum(percentage_error_500g)/length(mean_error_500g)
% 
% percentage_error_2kg = mean_error_2kg./Torque(1515:1699);
% mean_percentage_error_2kg = sum(percentage_error_2kg)/length(mean_error_2kg)
% 
% mean_error_5kg_2 = ((Torque2(1:306) - LoadCell2(1:306)).^2).^0.5;
% mean_error_500g_2 = ((Torque2(600:1050) - LoadCell2(600:1050)).^2).^0.5;
% mean_error_2kg_2 = ((Torque2(1515:2000) - LoadCell2(1515:2000)).^2).^0.5;
% 
% percentage_error_5kg_2 = mean_error_5kg_2./Torque2(1:306);
% mean_percentage_error_5kg_2 = sum(percentage_error_5kg_2)/length(mean_error_5kg_2)
% 
% percentage_error_500g_2 = mean_error_500g_2./Torque2(600:1050);
% mean_percentage_error_500g_2 = sum(percentage_error_500g_2)/length(mean_error_500g_2)
% 
% percentage_error_2kg_2 = mean_error_2kg_2./Torque2(1515:2000);
% mean_percentage_error_2kg_2 = sum(percentage_error_2kg_2)/length(mean_error_2kg_2)

%% PLOT FOR TEST 1
figure('Units', 'pixels', ...
    'Position', [100 100 600 300]);
grid on
hold on

% h1encoder = plot(Timems*ms_to_s, Encoder*qc_to_rad);
h1eposposition = plot(SampleNumber10000samplespersecond*0.05, X1*qc_to_rad);
% h1loadcell = plot(Timems*ms_to_s, LoadCell);
% h1torque = plot(Timems*ms_to_s, Torque);

set(h1eposposition                       , ...
  'LineWidth'       ,    2        , ...
  'LineStyle'       , '-'      , ...
  'color'          , Colors(2,:)     );

% set(h1torque                       , ...
% 'LineWidth'       ,    2        , ...
% 'LineStyle'       , '-'        , ...
% 'color'          , Colors(1,:)     );

h1Legend = legend( ...
  [h1eposposition], ...
  'LoadCell Data', ...
  'location', 'NorthEast' );

% TrackingTitle  = title ('My Publication-Quality Graphics');
h1YLabel = ylabel('Position (mNm)');
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

% saveas(gcf,'20170712-loadcell_tests_response.png')
