function [foriginal]=Sys_obj(x)
%[rProp,V,mBattery,MMotor,
%mtow,Ereserve,S, rpm, eta_motor,m_gb]
    %Pwing_output=smbo_1(x(5),x(7));%2 outputs (mtow,ss)
    %$Pmotor_output=smbo_2(x(8),x(9)); % 2 outputs (rpm,eta_motorm)
    %Pgearbox_output=smbo_3(x(1),x(2),x(5),x(7),x(8),x(9),x(10)); % 7 outputs (rest 7 outputs)
    %Pprop_output=Pprop(x(1),x(2),x(3),x(4),x(6),x(10)); % 3 outputs
    %misc.range=rangei;
    rangei=50000;
    misc.range=rangei;
    misc.vehicle='tiltwing';
    misc.payload=300;
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        
        
        
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;
        
        
        
       
        % For the nominal mission compute the energy use, flight time, hover
        % performance, and cruise performance
        [ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        % Compute operating cost
        C = operatingCost(vehicle,rProp,flightTime,ENominal,mass,cruiseOutput);
        
        %% Objective is operating cost per flight
        f = mtow;
        f=C.costPerFlight;
        
        
        
        %get original objective
        foriginal =f;
           
       
        
        
    end
