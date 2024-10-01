function [cn,ceqn] = constraints_Sys(x)
    Pwing_output=smbo_1(x(5),x(7));%2 outputs (mtow,ss)
    Pmotor_output=smbo_2(x(8),x(9)); % 2 outputs (rpm,eta_motorm)
    Pgearbox_output=smbo_3(x(1),x(2),x(5),x(7),x(8),x(9),x(10)); % 7 outputs (rest 7 outputs)
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
        Ereserve=x(6);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;

        ceqn(1) =  Pwing_output(3);
        ceqn(2) =  Pmotor_output(3);
        ceqn(3) =  Pgearbox_output(8);
       
        % Assumed values
        batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
        motorPowerDensity = 5;
        dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission
        
       [~,~,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        

        

        
        % Constraint on MTOW
        cn(1) = mass.W - mtow * 9.8;
        
        
        
        % Battery sizing
        [EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        cn(2) = EReserve - mBattery * batteryEnergyDensity * dischargeDepthReserve / 1000;
        
%         % motor power density
%         cn(4) = hoverOutput.PMax / 1000 - mMotors * motorPowerDensity;
        
        
        %motor sizing based on torque
        torq=0.74*(hoverOutput.PMax/8)/(rpm*2*pi/60);
        lb2kg = 0.453592;   
        cn(3) = 0.3928*(torq^0.8587)*lb2kg*8 - mMotors;
        
        
%         % Constraint on tip speed
%         tip_mach=0.65;
%         omega=tip_mach*340.294/rProp;
%         rpm_max=omega*60/(2*pi);
%         cn(4)=rpm-rpm_max;
%         cn(4)=0;
        

        
        % Ereserve constraint
        ceqn(4)=abs (EReserve-x(6)); 
    end
