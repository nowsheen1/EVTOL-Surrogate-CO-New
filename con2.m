
% f = OBJ(x)
%   f: Objective Function Value
%   x: Design Variables
function [ceqn] =con2(x,rpms, eta_motors)
        
        rpm=x(1);
        ceqn= x(2)-motor_eta(rpm);
        cn=[];
        
    end
