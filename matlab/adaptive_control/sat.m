function x = sat(x,limit)

% y = x_m(3,1)-2*lambda*x_til(2,1)-lambda^2*x_til(1,1);
% y = 30*sin(t);

for i = 1:length(x);
    if abs(x(i)) > limit    
    x(i) = limit*sign(x(i)); 
    else
        
    end
end

end