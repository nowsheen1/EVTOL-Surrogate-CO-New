
% f = OBJ(x)
%   f: Objective Function Value
%   x: Design Variables
   function [foriginal]=obj1(x,mtows,Ss) %changed this line
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
        rangei=50000;
        misc.range=rangei;
        misc.vehicle='tiltwing';
        misc.payload=300;
        mtow = x(1);
        S = x(2); %m2
        
        
        %get original objective
        %foriginal =mtow ;
        
        
        %Assemble c: target - response
        
        c(1) = diffc(mtows,mtow);
        c(2) = diffc(Ss,S);
        foriginal=0.5*sum(c);
        
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
        %fATC=foriginal; % changed this line
        
      
        
        
   end
   function cd= diffc(aT,aR)
       %cd=(aT-aR)/(aT) ; %target - response
       cd=(aT-aR).^2 ; %target - response
       %cd=0.5*(aT-aR).^2; %changed this line
       %c=abs(c);
    end
