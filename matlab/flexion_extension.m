function flexion_extension()

%**************************************************************************
% Autor:  Rafael Sanchez Souza
%------------ Escola Politecnica da Universidade de Sao Paulo -------------
%
% Version: 1.0
% Date: 10.25.2016 [mm.dd.yyyy]%
% Description:
%  Try flexion extension models
% Input files:
%	(None)
%
% Final Output files:
%   Not yet
%
%**************************************************************************
% Steps:
%   I. Modeling
%   II. ODE
%**************************************************************************

dt = 0.01;
T = 1;
t = 0:dt:T;
Colors = linspecer(6); 

%**************************************************************************
% I. Modeling
%--------------------------------------------------------------------------
%   
%   System is a simple spring-mass model
%   All Units are in SI
%   
%**************************************************************************

% human parameters (plant) {from Olaya cap5}
% j_h = 0.002; % Nms^2/grau
% b_h = 0.044; % Nms/grau [2.5-4.7]
% k_h = 0.15; % Nm/grau [5.3-13.1]
j_h = 0.115; % Nms^2/rad
b_h = 2.52; % Nms/rad [2.5-4.7]
k_h = 8.6; % Nm/rad [5.3-13.1]
L = 0.3; % m

% Musculo Humano MSM
% Jmsm = 0.010; % kgm^2
J_msm = 0.00171; % kgm^2
B_ce = 1.396; % Nms/degrees
% Bce = 80; % Nms/rad
K_ce = 13.96; % Nm/degrees
% Kce = 800; % Nm/rad
K_se = 0.1396;
% Kse = 8;

%**************************************************************************
% II. ODE
%--------------------------------------------------------------------------
%   
%   Differencial Equation Integration
%   All Units are in SI
%   
%**************************************************************************

[theta] = euler_test(dt,T,j_h,b_h,k_h,L,t);

%%%%%%%%%%%
% Figures %
%%%%%%%%%%%

figure

plot(t,theta(1,:))