
% f = OBJ(x)
%   f: Objective Function Value
%   x: Design Variables
function [ceqn] = con1(x,mtows,Ss)
   
        rangei=50000;
        misc.range=rangei;
        misc.vehicle='tiltwing';
        misc.payload=300;
        cn = [];
        rho = 1.225;
        % Specify stall conditions
        VStall = 35; % m/s
        CLmax = 1.1; % Whole aircraft CL, section Clmax much higher
        ceqn=x(2)-((x(1)*9.8) / (0.5 * rho * VStall^2 * CLmax));
        
    end

