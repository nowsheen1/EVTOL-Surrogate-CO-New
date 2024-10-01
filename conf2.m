
%
% pc = CONF()
%   pc: Structured Data Containing Problem Configurations

function pc = conf2()


    pc.nvar = 2;                % Number of variables
    pc.lb=[2600,0.01];
    pc.ub= [8800,1];
    pc.fs_g = 0.6;             % Shrink factor for global sample range
    %pc.xtrue = [0];           % True soltuion in x (for comparison)
    %pc.ftrue = 0;               % True solution in f (for comparison)
end