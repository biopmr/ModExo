function mrac_main(X0,Ahat0,jv,bv,kv,K,m_ext,T)

%**************************************************************************
% Autor:  Rafael Sanchez Souza
%------------ Escola Politecnica da Universidade de Sao Paulo -------------
%
% Version: 2.0
% Date: 15.08.2016 [mm.dd.yyyy]%
% Description
%  Model Reference Adaptative Control Shell
% Input files:
%	(None)
%
% Final Output files:
%   (None)
%
%**************************************************************************
% Steps:
%   I. Dinamica do Sistema
%   II. Filtro de Kalman no Tempo Cont?nuo com Medidas Discretas
%**************************************************************************

%**************************************************************************
% I. Din?mica do Sistema
%--------------------------------------------------------------------------
%   
%   As principais for?as que governam o movimento dos membros humano s?o
%   relativas ? din?mica de corpo r?gido dos m?sculos dos m?sculos.
%   Para efeitos deste exerc?cio, considera-se o membro como um sistema
%   massa-mola-amortecedor sujeito ? um torque na junta correspondente.
%
%   Analogamente, o exoesqueleto passivo (sem atua??o nos motores) ? tamb?m
%   descrito como um sistema massa-mola-amortecedor.
%
%   Finalmente, acomplam-se estas duas din?micas mais a din?mica simulada
%   pelo controlador para chegar-se ? din?mica final do sistema.
%   
%**************************************************************************

dt = 0.05; %samplong time of 50ms
t = 0:dt:T;
tt = linspace(-pi,2*pi,T/dt+1);
Colors = linspecer(6); 

n = 2; % system order
N = n + 1; % auxiliar variable

%% REFERENCES

% step
Ur(1,:) = LoadCell(50:395);

% % sin1
% Ur(1,:) = K*sin(2*pi/5*t);
% Ur(2,:) = K*(2*pi/5)*cos(2*pi/5*t);
% Ur(3,:) = -K*(2*pi/5)^2*sin(2*pi/5*t);

% % sin2
% Ur(1,:) = K^2*sin(t)+K*sin(10*t);
% Ur(2,:) = K^2*cos(t)+K*(10)*cos(10*t);
% Ur(3,:) = -K^2*sin(t)+-K*(10)^2*sin(10*t);

% % square
% Ur(1,:) = K*square(t*2*pi/5);

% % Smoothsquare
% Ur(1,:) = K*tanh(6*sin(1*t));

% % sequence of steps 
% Ur(1,1:501) = 0.5;
% Ur(1,502:1001) = 1.0;
% Ur(1,1002:1501) = 1.5;
% Ur(1,1502:2001) = 2.0;
% Ur(1,2002:2501) = 10;

%% ODE

[x_m,x,x_1,Ue,Ahat,v,e,G,V,j_eq,b_eq,k_eq,C,J] = exo_horizontal(N,Ur,X0,dt,T,t,jv,bv,kv,Ahat0,m_ext);
% [x_m,x,Ue,Ahat,v,e,G,V,j_eq,b_eq,k_eq,C,J] = rk_exo(N,Ur,X0,dt,T,t,jv,bv,kv,Ahat0,m_ext);

%% TRACKING PERFORMANCE PLOT
figure('Units', 'pixels', ...
    'Position', [100 100 600 300]);
% figure;
hold on;
hold on
grid on

hold on
h1plant = plot(t,x(1,:));
h1plant1 = plot(t,x_1(1,:));
h1model = plot(t,x_m(1,:));
% h1input=plot(t,Ur(1,:));

set(h1plant                       , ...
  'LineWidth'       ,    3        , ...
  'LineStyle'       , '-'      , ...
  'color'          , Colors(2,:)     );

set(h1plant1                       , ...
  'LineWidth'       ,    3        , ...
  'LineStyle'       , '--'        , ...
  'color'          , Colors(1,:)     );


set(h1model                       , ...
  'LineWidth'       ,    3        , ...
  'LineStyle'       , '--'        , ...
  'color'          , Colors(2,:)     );

% set(h1input                       , ...
%   'LineWidth'       ,    2        , ...
%   'LineStyle'       , '-'        , ...
%   'color'          , Colors(6,:)     );

h1Legend = legend( ...
  [h1plant, h1plant1, h1model], ...`
  '\theta_{EQ2}', ...
  '\theta_{EQ}'           , ...
  '\theta_m'              , ...
  'location', 'SouthEast' );

% TrackingTitle  = title ('My Publication-Quality Graphics');
h1YLabel = ylabel('\theta (rad)');
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

% axis([0 T 0 max(x(1,:)*1.1)])

% saveas(gcf,'03_sin_sin_response.png')
% savefig('03_sin_sin_response')

%% CONTROL EFFORT PLOT
% figure
% hold on
% grid on
% 
% plot(t,Ue(1,:),'k','LineWidth',4);
% % legend('\mu = 1','\mu = 0.5','\mu = 2.12')
% % plot(tt,ub,'b','LineWidth',4);
% % plot(t,ua,'k','LineWidth',4);
% % 
% title('Control Effort')
% xlabel('time[s]')
% ylabel('U_e[Nm]')
% 
% % saveas(gcf,'03_control_effort.png')
% % savefig('03_control_effort')

%% PARAMETER ESTIMATION PLOT
% 
% auxAhat(1:3,1:T/dt+1) = Ahat(1,:,:);
% 
% subplot(1,2,2)
% hold on
% grid on
% 
% h2inertia = plot(t,j_eq*ones(T/dt+1,1));
% % h2inertia = plot(t,J);
% h2Ahat1 = plot(t,auxAhat(1,:));
% h2damping = plot(t,b_eq*ones(T/dt+1,1));
% % h2damping = plot(t,C);
% h2Ahat2 = plot(t,auxAhat(2,:));
% h2stiffness = plot(t,k_eq+G);
% h2Ahat3 = plot(t,auxAhat(3,:));
%  
% set(h2inertia                     , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '--'        , ...
%   'color'          , Colors(3,:)     );
% 
% set(h2damping                     , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '--'        , ...
%   'color'          , Colors(4,:)     );
% 
% set(h2stiffness                   , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '--'        , ...
%   'color'          , Colors(5,:)     );
% 
% set(h2Ahat1                       , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '-'         , ...
%   'color'          , Colors(3,:)     );
% 
% set(h2Ahat2                       , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '-'         , ...
%   'color'          , Colors(4,:)     );
% 
% set(h2Ahat3                       , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '-'         , ...
%   'color'          , Colors(5,:)     );
% 
% h2Legend = legend( ...
%   [h2inertia, h2Ahat1, h2damping, h2Ahat2, h2stiffness, h2Ahat3], ...
%   'A1'                 , ...
%   'A1hat'              , ...
%   'A2'                 , ...
%   'A2hat'              , ...
%   'A3'                 , ...
%   'A3hat'              , ...
%   'location', 'SouthEast' );
% 
% % TrackingTitle  = title ('My Publication-Quality Graphics');
% h2YLabel = ylabel('parameter estimation');
% h2XLabel = xlabel('time (s)');
% 
% 
% set([h2Legend, gca]             , ...
%     'FontSize'   , 13           );
% 
% set( gca                       , ...
%     'FontSize',         15     , ...
%     'FontName'   , 'Helvetica' );
% 
% set(gca, ...
%   'XMinorTick'  , 'on'      , ...
%   'YMinorTick'  , 'on'      , ...
%   'YGrid'       , 'off'      , ...
%   'XGrid'       , 'off'      , ...
%   'LineWidth'   , 1         );
% 
% set([h2XLabel, h2YLabel], ...
%     'FontName'   , 'AvantGarde');

%% INPUT PLOT
% subplot(1,3,2)
% hold on
% grid on
% inputplot=plot(t,Ur);
% 
% set(inputplot                     , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '-'        , ...
%   'color'          , Colors(6,:)     );
% 
% % TrackingTitle  = title ('My Publication-Quality Graphics');
% inputYLabel = ylabel('U_r (Nm)');
% inputXLabel = xlabel('time (s)');
% 
% set( gca                       , ...
%     'FontSize',         15     , ...
%     'FontName'   , 'Helvetica' );
% 
% set(gca, ...
%   'XMinorTick'  , 'on'      , ...
%   'YMinorTick'  , 'on'      , ...
%   'YGrid'       , 'off'      , ...
%   'LineWidth'   , 1         );
% 
% set([inputXLabel, inputYLabel], ...
%     'FontName'   , 'AvantGarde');

%% TRACKING ERROR PLOT
 
% figure
% hold on
% grid on
% 
% h3error1 = plot(t,e(1,:));
% % h3error2 = plot(t,e(1,:));
% % h3error3 = plot(t,e(1,:));
% % h3error4 = plot(t,e(1,:));
% 
% set(h3error1                     , ...
%   'LineWidth'       ,    3        , ...
%   'LineStyle'       , '-'        , ...
%   'color'          , Colors(1,:)     );
% 
% % set(h3error2                     , ...
% %   'LineWidth'       ,    3        , ...
% %   'LineStyle'       , '-'        , ...
% %   'color'          , Colors(4,:)     );
% % 
% % set(h3error3                     , ...
% %   'LineWidth'       ,    3        , ...
% %   'LineStyle'       , '-'        , ...
% %   'color'          , Colors(5,:)     );
% % 
% % set(h3error4                     , ...
% %   'LineWidth'       ,    3        , ...
% %   'LineStyle'       , '-'        , ...
% %   'color'          , Colors(6,:)     );
% 
% % h2Legend = legend( ...
% %   [h3error1, h3error2, h3error3, h3error4], ...
% %   'Step 1'                 , ...
% %   'Step 2'              , ...
% %   'Step 3'                 , ...
% %   'Step 4'              , ...
% %   'location', 'SouthEast' );
% 
% % TrackingTitle  = title ('My Publication-Quality Graphics');
% inputYLabel = ylabel('tracking error');
% inputXLabel = xlabel('time (s)');
% 
% set( gca                       , ...
%     'FontSize',         15     , ...
%     'FontName'   , 'Helvetica' );
% 
% % set([h2Legend, gca]             , ...
% %     'FontSize'   , 13           );
% 
% set(gca, ...
%   'XMinorTick'  , 'on'      , ...
%   'YMinorTick'  , 'on'      , ...
%   'YGrid'       , 'off'      , ...
%   'LineWidth'   , 1         );
% 
% set([inputXLabel, inputYLabel], ...
%     'FontName'   , 'AvantGarde');

%% SAVING FILES

% axis([0 T 0 20])
% 
% saveas(gcf,'04_step_20_percent_uncertain_exoskeleton_compensation.png')
% savefig('04_step_20_percent_uncertain_exoskeleton_compensation')

% saveas(gcf,'05_step_0_knowledge_assistance.png')
% savefig('05_step_0_knowledge_assistance')

% saveas(gcf,'06_tanh_0_knowledge_assistance.png')
% savefig('06_tanh_0_knowledge_assistance')

% saveas(gcf,'07_sin_0_knowledge_assistance.png')
% savefig('07_sin_0_knowledge_assistance')

% saveas(gcf,'08_square_unmodelled.png')
% savefig('08_square_unmodelled')

% saveas(gcf,'09_sin_error.png')
% savefig('09_sin_error')

end