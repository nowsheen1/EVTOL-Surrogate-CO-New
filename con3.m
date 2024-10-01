
% f = OBJ(x)
%   f: Objective Function Value
function [ceqn] = con3(x,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg)

        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        
         rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;        vehicle=misc.vehicle;
        range=misc.range;
        
        
        
        [~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
       ceqn= x(7)-mass_gb(rpm,rProp,hoverOutput.PMax);
       
    end


