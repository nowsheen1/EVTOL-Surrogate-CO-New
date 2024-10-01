
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    function [foriginal]=obj3(x,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg)
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
         rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;
        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        %get original objective
         %foriginal =m_gb ;
        
        %Assemble c: target - response
        
        c(1) = diffc(rPropg,rProp);
        c(2) = diffc(Vg,V);
        c(3) = diffc(mtowg,mtow);
        c(4) = diffc(Sg,S) ;
        c(5) = diffc(rpmg,rpm);
        c(6) = diffc(eta_motorg,eta_motor);
        c(7) = diffc(m_gbg,m_gb);%Assemble v and w.Make it consistently ordered
        foriginal=(1/7)*sum(c);
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
        
        
 
    end

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
   % function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
        
    %    rProp=x(1);
     %   V=x(2);
      %  mtow=x(3);
       % S=x(4);
        %rpm=x(5);
        %eta_motor=x(6);
        %m_gb=x(7);
        
        
        %vehicle=misc.vehicle;
        %range=misc.range;
        
        
        
        %[~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        %ceqn= x(7)-mass_gb(rpm,rProp,hoverOutput.PMax);
        %cn=[];
    %end


    function cd= diffc(aT,aR)
        %cd=0.5*(aT-aR).^2; %target - response
           cd=(aT-aR).^2;
        %                c=abs(c);
    end

   
