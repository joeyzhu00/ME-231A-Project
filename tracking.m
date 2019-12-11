function [Uopt, Xopt, Jopt] = tracking(x0, Ytar)
% this function should track the target trajectory Y
% Y is 4 by N matrix given by the high level path planner

% the cost function should be like this:
%   min       sum 0 to N 2-norm(C * x_k - Y_k)^2 + deltaUk'*R*deltaUk + f(alpha)
% Uk,alpha

% s.t.            x_k+1 = A * x_k + B * u_k  k = 0,1,2,....,N
%                 y_k = C * x_k              k = 0,1,2,....,N
%                 Ac * x_k <= alpha          k = 0,1,2,....,N
%                 u_k belong to U            k = 0,1,2,...,CH
%                 deltau_k = u_k - u_k-1     k = 0,1,2,...,CH
%                 u_-1 = 0
%                 u_k = U_k-1                k = CH+1,...,N 
%                 x_0 = x0

% here N is the prediction horizon,
% CH is the control horizon
% the model should be a linearized model around that speed point
% hard constrain on input and soft constrain on output
% the problem will always be feasible
end