function [x_m,x,x_1,Ue,Ahat,v,e,G,V,j_eq,b_eq,k_eq,C,J] = exo_gravity(N,Ur,X0,dt,T,t,jv,bv,kv,Ahat0,EPOSPosition,X1,X2,X3,LoadCell)

% control parameters
zeta = 0.8;
wn = 5; % rad/s
% wd = sqrt(1-zeta^2)
% beta = atan(wn*wd/zeta)
% tr = (pi - beta)/(wn*sqrt(1-zeta^2))

a0 = 1;
a1 = 2*zeta/wn;
a2 = 1/wn^2;

sat_limit = 24;

% human parameters (plant) {from Olaya cap5}
% j_h = 0.002; % Nms^2/grau
% b_h = 0.044; % Nms/grau [2.5-4.7]
% k_h = 0.15; % Nm/grau [5.3-13.1]
j_h = 0.115; % Nms^2/rad
b_h = 2.52; % Nms/rad [2.5-4.7]
k_h = 8.6; % Nm/rad [5.3-13.1]
L = 0.3; % m

% % exoskeleton parameters (plant)
% %General Module Specification
% R = 0.045;                      % joint radius (same as maxon motor)
% L = 0.13;                       % link lengh (GUESS!! Must check!)
% B = 0.035;                      % link width (GUESS!! Must check!)
% 
% var_c = 1;                      % variation of friction coeficient
% c = 0.15*var_c;                 % friction coeficient
% c1 = c;
% c2 = c;
% m_m = 0.6;                      % mass of motor
% m_h = 0.240;                    % mass of harmonic drive
% m_st = 0.44 ;                   % mass of supporting alluminium structure
% m_l = 0.320;                    % link mass
% 
% I_m = 0.003060;                 % moment of inertia from maxon catalogue
% I_h = 0.282*10^(-4);            % moment of inertia of harmonic drive
% I_st = m_st*R^2/2;              % calculates moment of inertia of structure
% % I_st = m_st*L^2/3;      % calculates moment of inertia of structure DOUBLE CHECK!!!!!
% 
% I_j = I_m + I_h + I_st;        % moment of inertia of exoskeleton joint
% I_l = m_l*((L^2+B^2)/12 + (R+L/2)^2); % moment of inertia of exoskeleton link
% j_exo = I_j + I_l;           % moment of inertia of exoskeleton

% aguirre ollinguer
% Thus our values for the parameters for the exoskeleton mechanism were Ie = 0.199 kg-m, be = 1.32 N-m-s/rad, ke = 5.12 N-m/rad
j_exo = 0.199; % Nms^2/rad
b_exo = 1.32; % Nms/rad [2.5-4.7]
k_exo = 5.12; % Nm/rad [5.3-13.1]
L = 0.3; % m

% coupled system (plant)
j_eq = j_h + j_exo; % Nms^2/rad
% % j_eq = j_h; % Nms^2/rad
b_eq = b_h + b_exo; % Nms/rad 
k_eq = k_h + k_exo; % Nm/rad
L = 0.3; % m

% virtual parameters (reference model)
kv = 10; % parameter used in test
bv = 1; % parameter used in test
jv = 20; % parameter used in test

a0 = kv;
a1 = bv;
a2 = jv;
A = [a2 a1 a0]';     %size N

% error convergence
b0 = 40;
b1 = 10;
B = [b1 b0]; % size N

% variables initialization
Ahat = zeros(2,3,T/dt+1);     % estimated parametes (size N)
Atil = zeros(2,3,T/dt+1); % tracking error (size N)
x = zeros(3,T/dt+1); % initiates the system states [x, dx, ddx] array
x_1 = zeros(3,T/dt+1); % initiates the system states [x, dx, ddx] array
x_m = zeros(3,T/dt+1); % initiates reference model states [xm, dxm, ddxm] array
e = zeros(2,T/dt+1); % initiates the Lyapunov Function [x_til, dx_til] array
V = zeros(2,T/dt+1); % initiates the tracking error [x_til, dx_til] array
Ue = zeros(3,T/dt+1); % initiates exoskeleton control array
v = zeros(3,T/dt+1); % initiates signal variable array

% initial conditions
x(1,1) = X1(1);
x(2,1) = X2(1);
x(3,1) = X3(1);
x_1(:,1) = X0;
x_m(:,1) = X0;
e(1,1) = x(1,1) - x_m(1,1);
e(2,1) = x(2,1) - x_m(2,1);
Ahat(1,:,1) = Ahat0; % Ahat is transposed from defined in Slotine
z(1,1) = x_m(3,1) - dot_product(B,[e(2,1) e(1,1)],N-1);
v(:,1) = [z(1,1) x(2,1) x(1,1)];
Ue(1,1) = Ahat(1,1)*v(1,1);
% G(1) = j_eq*L/2*cos(x(1,1)*pi/180); % non linearity due to gravity
% C(1) = b_eq*1.15*sin(x(1,1)*pi/180); % non linearity due to coriolis / friction (MUST RE DO)
% J(1) = j_eq*(1+abs(0.5*sin(t(1)))); 
G(1) = 0;
C(1) = 0;
J(1) = 0;

for i = 1:(T/dt);
    % tracking error
    e(1,i+1) = x(1,i) - x_m(1,i);
    e(2,i+1) = x(2,i) - x_m(2,i);
    
    % signal (Slotine, eq 8.38)
    z(1,i+1) = x_m(3,i) - dot_product(B,[e(2,i) e(1,i)],N-1);
    v(:,i+1) = [z(1,i) x(2,i) x(1,i)]';
    
    % control law (Slotine, eq 8.39)
    Ue(1,i+1) = dot_product(v(:,i),Ahat(1,:,i),N);                          % Control Law
    Ue(1,i+1) = sat(Ue(1,i+1),sat_limit);                                   % With saturation
    
    % reference model    
    x_m(1,i+1) = x_m(1,i) + dt*x_m(2,i);
    x_m(2,i+1) = x_m(2,i) + dt*x_m(3,i);
    x_m(3,i+1) = 1/A(1)*(-A(2)*x_m(2,i) - A(3)*x_m(1,i) + Ur(1,i));
%     x_m(3,i+1) = 1/A(1)*(-A(2)*x_m(2,i) - A(3)*x_m(1,i) + 1);
    
    % planta sem controle
    x_1(1,i+1) = x_1(1,i) + dt*x_1(2,i);
    x_1(2,i+1) = x_1(2,i) + dt*x_1(3,i);
    x_1(3,i+1) = 1/j_eq*(-b_eq*x_1(2,i) - k_eq*x_1(1,i) + Ur(1,i));
    
    % plant
    x(1,i) = X1(i,1);
    x(2,i) = X2(i,1);
    x(3,i) = X3(i,1);

    % error
    Atil(1,:,i+1) = Ahat(1,:,i) - A';
    Atil(2,:,i+1) = Ahat(2,:,i) - A';
    
    % choice of adaptation Law
    a = [0 1 ; ...
         -B(1) -B(2)];
     
    b = [0 1]';
    
    Gamma = [0.1 0 0; ... % symmetric matrix
             0 50 0; ...
             6 0 400]; %original 400
           
    PP = 0.1;     
    
    P11 = 0; % doesn't matter
    P12 = 0; % doesn't matter
    P21 = 3; % convergence "gain"
    P22 = 1; % convergence "stifness"
    
    P = PP*[P11 P12; ... % symmetric matrix
            P21 P22];    
        
    Q = - (P*a + a'*P);

    % states computation (not needed)
%     e_d(1,i+1) = e(1,i) + dt*e(2,i);
%     e_d(2,i+1) = dot_product(b*1/3*v(:,i)',Atil(1,:,i),N);
    
    % lyapunov
    V(1,i+1) = e(:,i)'*P*e(:,i) + Atil(1,:,i)*inv(Gamma)*Atil(1,:,i)';
%     V(2,i+1) = e(:,i)'*Q'*e(:,i) + 2*Atil(1,:,i)*v(1,i)*b*P*e(1,i) + 2*Atil(1,:,i)*inv(Gamma)*Atil(2,:,i)';
    V(2,i+1) = -e(:,i)'*Q*e(:,i);
    
    % parameters dynamics
    Ahat(1,:,i+1) = Ahat(1,:,i) + dt*Ahat(2,:,i);
    Ahat(2,:,i+1) = -Gamma*v(:,i)*b'*P*e(:,i);   

    
end

end
