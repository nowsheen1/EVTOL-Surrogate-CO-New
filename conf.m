
% pc = CONF()
%   pc: Structured Data Containing Problem Configurations

function pc = conf()


    pc.nvar = 2;                % Number of variables
    pc.lb=[100,1];
    pc.ub= [9999,30];
   
    pc.fs_g = 0.6;             % Shrink factor for global sample range
    %pc.xtrue = [0];           % True soltuion in x (for comparison)
    %pc.ftrue = 0;               % True solution in f (for comparison)
end