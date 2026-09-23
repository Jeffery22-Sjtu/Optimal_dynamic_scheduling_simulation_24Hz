function [sys,x0,str,ts,simStateCompliance] = EKFsfun3(t,x,u,flag,delta_t,delta_tc,LiniEKF,TraceS,A1,R1,A2,R2,AR,alpha)


switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(LiniEKF);

  case 1,
    sys=mdlDerivatives(t,x,u);

  case 2,
    sys=mdlUpdate(t,x,u,delta_t,delta_tc,TraceS,A1,R1,A2,R2,AR,alpha);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(LiniEKF)


sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 4;
%x(1)=L1
%x(2)=L2
%x(3)=LR
%x(4)=count
sizes.NumOutputs     = 3;   % L1, L2, LR
sizes.NumInputs      = 3;   % L2=u(1),u=u(2),theta=u(3)
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0=[LiniEKF';1];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)

sys = [];

function sys=mdlUpdate(t,x,u,delta_t,delta_tc,TraceS,A1,R1,A2,R2,AR,alpha)
%Common constant
rho=1000;                   %density of water kg/m3
g=9.8;                      %gravity constant, m/s
%parameters of BASIN
Q=0.5*eye(3,3);             %predict accuracy
R=1;                        %observe accuracy
A=1/(A1*rho);
B=1/(A2*rho);
C=1/(AR*rho);
D=sqrt(rho*g)/(R1*rho);
E=sqrt(rho*g)/(R2*rho);

global jX3 bX3 bP3 jP3 bZ3 Kk3
jX3=[x(1);x(2);x(3)];
L_M=u(1);%input L2
U=u(2);
theta=u(3);
if (rem(x(4),delta_tc/delta_t)==0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% EKF Algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
E=E*theta;

bX3=[jX3(1)-A*D*delta_tc*sqrt(jX3(1))+delta_tc*A*alpha*U;delta_tc*B*D*sqrt(jX3(1))+jX3(2)-delta_tc*B*E*sqrt(jX3(2));delta_tc*C*E*sqrt(jX3(2))-delta_tc*C*alpha*U+jX3(3)]; 
AJ=[1-delta_tc*A*D*1/(2*sqrt(jX3(1))) 0 0;
   delta_tc*B*D*1/(2*sqrt(jX3(1)))   1-delta_tc*B*E*1/(2*sqrt(jX3(2))) 0;
   0 delta_tc*C*E/(2*sqrt(jX3(2))) 1];

bP3=AJ*jP3*AJ'+Q;
H=[0 1 0];
Kk3=bP3*H'*(inv(H*bP3*H'+R));
bZ3=bX3(2);  
jX3=bX3+Kk3*(L_M-bZ3)*TraceS(x(4)); 
jP3=bP3-Kk3*H*bP3;
end
x(1)=jX3(1);

if x(1)<=0
    x(1)=0.0001;
end
x(2)=jX3(2);
if x(2)<=0
    x(2)=0.0001;
end
x(3)=jX3(3);
if x(3)<=0
    x(3)=0.0001;
end
x(4)=x(4)+1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys = [x];


function sys=mdlOutputs(t,x,u)

sys = [x(1);x(2);x(3)]; %L1,L2

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;             %Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
