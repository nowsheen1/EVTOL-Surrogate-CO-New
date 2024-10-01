
%
% pc = CONF()
%   pc: Structured Data Containing Problem Configurations

function pc = conf3()


    pc.nvar = 7;                % Number of variables
    pc.lb=[0.01,10,100,1,2600,0.01,20];
    pc.ub=[10, 100, 9999,30,8800,1,200];
    pc.fs_g = 0.6;             % Shrink factor for global sample range
    %pc.xtrue = [0];           % True soltuion in x (for comparison)
    %pc.ftrue = 0;               % True solution in f (for comparison)
end