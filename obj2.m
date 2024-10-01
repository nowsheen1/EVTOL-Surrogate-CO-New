
% f = OBJ(x)
%   f: Objective Function Value

 function [foriginal]=obj2(x,rpms,eta_motors)
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
         rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;
        rpm = x(1);
        eta_motor = x(2); %m2
        
        
        
        
        %get original objective
        %foriginal =eta_motor ;
        
        
        %Assemble c: target - response
        
        c(1) = diffc(rpms,rpm);
        c(2) = diffc(eta_motors,eta_motor);
        foriginal=0.5*sum(c);
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
            
        
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
   % function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
    %    rpm=x(1);
     %   ceqn(1)= x(2)-motor_eta(rpm);
      %  cn=[];
        
    %end


    function cd= diffc(aT,aR)
        %cd=0.5*(aT-aR).^2 ; %target - response
           cd=(aT-aR).^2 ;
        %                c=abs(c);
    end





