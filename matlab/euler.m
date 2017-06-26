function [theta] = euler(dt,T,j_h,b_h,k_h,L,t)

% variables initialization
theta = zeros(3,T/dt+1); % initiates the system states [x, dx, ddx] array

% initial conditions
theta0 = pi/4;
theta(1,1) = 0;

for i = 1:(T/dt)
    
    % plant
    theta(1,i+1) = theta(1,i) + dt*theta(2,i);
    theta(2,i+1) = theta(2,i) + dt*theta(3,i);
    theta(3,i+1) = 1/j_h*(-b_h*theta(2,i)-k_h*(theta0 - theta(1,i))); %    without b_eq variation
    
end

end
