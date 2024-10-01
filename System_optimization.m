FUN = @Sys_obj;
global count count1;
count=0;
count1=0;
x0s=[8, 80, 290, 300, 650,1,1,5000,0.9,50];
 
lb=[0.01,10,50,20,100,1,1,2600,0.01,20];
ub=[10, 100, 999, 999, 9999,300,30,8800,1,200];
A =[];
b = [];
Aeq = [];
beq = [];
NONLCON = @constraints_Sys;


options=optimoptions('fmincon','MaxFunEvals' ,300,'MaxIter' ,1000,'TolX',1e-6,'TolFun',1e-6,'TolCon',10^-2,'Display','iter','FiniteDifferenceStepSize',10^-8);
%options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1500) ;% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = fmincon(FUN,x0s,A,b,Aeq,beq,lb,ub,NONLCON,options);

%old options
%options=optimoptions('fmincon','MaxFunEvals' ,300,'MaxIter' ,1000,'TolX',1e-6,'TolFun',1e-6,'TolCon',10^-2,'Display','iter','FiniteDifferenceStepSize',10^-8);

z=count;
h=count1;