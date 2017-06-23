function [x_m,x,x_1,Ue,Ahat,V,e,G] = pendulo(N,j_h,b_h,k_h,L,A,Ahat,Ahat0,Atil,B,Ur,Ue,V,X0,x,x_m,e,dt,T,t,sat_limit)

% initial conditions
x(:,1) = X0;
x_1(:,1) = X0;
x_m(:,1) = X0;
e(1,1) = x(1,1) - x_m(1,1);
e(2,1) = x(2,1) - x_m(2,1);
Ahat(1,:,1) = Ahat0;
z(1,1) = x_m(3,1) - dot_product(B,[e(2,1) e(1,1)],N-1);
V(:,1) = [z(1,1) x(2,1) x(1,1)];
Ue(1,1) = Ahat(1,1)*V(1,1);
% G(1) = j_h*L/2*cos(x(1,1)*pi/180);
G(1) = 0;

for i = 1:(T/dt);
    % non linearity
%     G(i+1) = j_h*L/2*cos(x(1,i)*pi/180);
    G(i+1) = 0;
    
    % tracking error
    e(1,i+1) = x(1,i) - x_m(1,i);
    e(2,i+1) = x(2,i) - x_m(2,i);
    
    % signal (Slotine, eq 8.38)
    z(1,i+1) = x_m(3,i) - dot_product(B,[e(2,i) e(1,i)],N-1);
    V(:,i+1) = [z(1,i) x(2,i) x(1,i)]';
    
    % control law (Slotine, eq 8.39)
    Ue(1,i+1) = dot_product(V(:,i),Ahat(1,:,i),N);                          % Control Law
%     Ue(1,i+1) = sat(Ue(1,i+1),sat_limit);                                   % Saturation
    
    % reference model    
    x_m(1,i+1) = x_m(1,i) + dt*x_m(2,i);
    x_m(2,i+1) = x_m(2,i) + dt*x_m(3,i);
    x_m(3,i+1) = 1/A(1)*(-A(2)*x_m(2,i) - A(3)*x_m(1,i) + Ur(1,i));
%     x_m(3,i+1) = 1/A(1)*(-A(2)*x_m(2,i) - A(3)*x_m(1,i) + 1);
    
    % planta sem controle
    x_1(1,i+1) = x_1(1,i) + dt*x_1(2,i);
    x_1(2,i+1) = x_1(2,i) + dt*x_1(3,i);
    x_1(3,i+1) = 1/j_h*(-b_h*x_1(2,i) - k_h*x_1(1,i) + Ur(1,i));

    % plant
    x(1,i+1) = x(1,i) + dt*x(2,i);
    x(2,i+1) = x(2,i) + dt*x(3,i);
    x(3,i+1) = 1/j_h*(-b_h*x(2,i) - k_h*x(1,i) - G(i) + Ue(1,i));

    % error
    Atil(1,:,i+1) = Ahat(1,:,i) - A';
    
    % adaptation law
    
    % closed loop error dynamics
    a = [0 1 ; ...
         -B(1) -B(2)];
     
    b = [0 1]';
    
    Gamma = 1*[1 0 0; ... % symmetric matrix
               0 1 0; ...
               0 0 1];
           
    PP = 1;     
    
    P = PP*[1 1; ... % symmetric matrix
            1 1];    
         
%     e(1,i+1) = e(1,i) + dt*e(2,i);
%     e(2,i+1) = e(2,i) + dt*e(3,i);
%     e(3,i+1) = b*1/A(3)*V(:,i)'*Atil(:,i);
    
    Ahat(1,:,i+1) = Ahat(1,:,i) + dt*Ahat(2,:,i);
    Ahat(2,:,i+1) = -Gamma*V(:,i)*b'*P*e(:,i);
    
end

end
