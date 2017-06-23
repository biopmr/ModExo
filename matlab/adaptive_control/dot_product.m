function [result] = dot_product(v, u, n)

    result = 0;
    for i = 1:n;
        result = result + v(i)*u(i);
    end
    
end

% original code, from: http://stackoverflow.com/questions/20733590/dot-product-function-in-c-language

% double dot_product(double v[], double u[], int n)
% {
%     double result = 0.0;
%     for (int i = 0; i < n; i++)
%         result += v[i]*u[i];
%     return result;
% }