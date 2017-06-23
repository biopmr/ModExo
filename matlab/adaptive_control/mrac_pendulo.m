function mrac_pendulo(X0,Ahat0,jv,bv,kv,K)

%**************************************************************************
% Autor:  Rafael Sanchez Souza
%------------ Escola Politecnica da Universidade de Sao Paulo -------------
%
% Version: 1.0
% Date: 15.08.2016 [mm.dd.yyyy]%
% Description
%  Adaptative Control for Human Limb and Exoskeleton Cooperation
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

dt = 0.01;
T = 10;
t = 0:dt:T;
tt = linspace(-pi,2*pi,T/dt+1);
Colors = linspecer(6); 

sat_limit = 24;

% MRAC DESIGN

n = 2; % system order
N = n + 1; % auxiliar variable

% pendule parameters (plant)
j_h = 3; % Nms^2/grau
b_h = 0; % Nms/grau [0.44-0.8]
k_h = 0; % Nm/grau [0.9-0.22]
L = 0.3; % m

% virtual parameters (reference model)
a0 = kv;
a1 = bv;
a2 = jv;
A = [a2 a1 a0]';     %size N

% adaptation parameters
Ahat = zeros(2,3,T/dt+1);     %size N

% error convergence
b0 = 5;
b1 = 5;
B = [b1 b0]; % size N

% variables initialization
Atil = zeros(2,3,T/dt+1); %size N
x = zeros(3,T/dt+1); % initiates an array of the system states [x, dx, ddx]
e = zeros(2,T/dt+1); % initiates an array of the tracking error [x_til, dx_til]
x_m = zeros(3,T/dt+1); % initiates an array of the reference model states [xm, dxm, ddxm]
Ue = zeros(3,T/dt+1); % initiates an array of exoskeleton control
V = zeros(3,T/dt+1); % initiates an array of v

% reference setpoints
% Ur(1,:) = 0;
% Ur(2,:) = 0;

Ur(1,:) = K*sin(2*pi/5*t);
% Ur(2,:) = K*(2*pi/5)*cos(2*pi/5*t);
% Ur(3,:) = -K*(2*pi/5)^2*sin(2*pi/5*t);

% xd(1,:) = A;
% xd(2,:) = 0;

% xd(1,:) = 0.52*sin(2*pi/5*t) + 1.04*sin(2.5*pi/5*t) + 2*sin(0.5*pi/5*t);
% xd(2,:) = 0.52*(2*pi/5)*cos(2*pi/5*t) + 1.04*(2.5*pi/5)*cos(2.5*pi/5*t) + 2*(0.5*pi/5)*cos(0.5*pi/5*t);

% xd(1,:) = A*square(tt);

[x_m,x,x_1,Ue,Ahat,V,e,G] = pendulo(N,j_h,b_h,k_h,L,A,Ahat,Ahat0,Atil,B,Ur,Ue,V,X0,x,x_m,e,dt,T,t,sat_limit);

% time response
figure
hold on
grid on

plot(t,x(1,:),'color',Colors(2,:),'LineWidth',3);
plot(t,x_1(1,:),'color',Colors(2,:),'LineWidth',3,'LineStyle', '--');
% plot(t,Ue(1,:),'LineWidth',2);
plot(t,x_m(1,:),'color',Colors(1,:),'LineWidth',3);
% plot(t,Ur(1,:),'LineWidth',2);

legend('\theta','\theta_m') 
% legend('\theta','Ue','\theta_m','\theta_d') 

title('Resposta da Planta no Tempo para B = [6 35]')
xlabel('tempo (s)')
ylabel('\theta')

% saveas(gcf,'03_resposta_planta_tempo_lambda_B_6_35_degrau.png')

% % % control effort
% %  
% % figure
% % hold on
% % grid on
% % 
% % plot(t,u,'k','LineWidth',2);
% % % legend('\mu = 1','\mu = 0.5','\mu = 2.12')
% % % plot(tt,ub,'b','LineWidth',2);
% % % plot(t,ua,'k','LineWidth',2);
% % % 
% % title('Control Effort')
% % xlabel('tempo (s)')
% % ylabel('u(t)')
% % 
% % % saveas(gcf,'chattering2.png')

% % error
%  
% figure
% hold on
% grid on
% 
% plot(t,e(1,:),'LineWidth',2);
% plot(t,x(1,:),'LineWidth',2);
% legend('erro','\theta');
%  
% title('Tracking Error')
% xlabel('tempo (s)')
% ylabel('\theta')
% % 
% % % saveas(gcf,'b4_erro_lambda_5_gamma_17.png')

% estimative

auxAhat(1:3,1:T/dt+1) = Ahat(1,:,:);

figure
hold on
grid on

plot(t,j_h*ones(T/dt+1,1),'b:','LineWidth',2);
plot(t,auxAhat(1,:),'b','LineWidth',2);
plot(t,b_h*ones(T/dt+1,1),'r:','LineWidth',2);
plot(t,auxAhat(2,:),'r','LineWidth',2);
plot(t,k_h*ones(T/dt+1,1),'y:','LineWidth',2);
plot(t,auxAhat(3,:),'y','LineWidth',2);

legend('A1','A1hat','A2','A2hat','A3','A3hat');
 
title('Estimativa')
xlabel('tempo (s)')
ylabel('')
% 
% % saveas(gcf,'b4_estimativa_lambda_5_gamma_17.png')



end