%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [CONFIDENTIAL]
% S-function of the controller of loop 2 and loop 4
% Objective: (1) generate control command
% (2) make 1 step prediction
% Control rate: 24 Hz
% Refer to https://github.com/WU-CPSL/WCPSv3_docker
% The multi-rate implementation refers to WCPSv4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts,simStateCompliance] = Controllersfun2(t,x,u,flag,delta_t,delta_tc,A1,R1,A2,R2,alpha,uini,trigger,L1sp,L2sp,usp)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(uini);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u,delta_t,delta_tc,A1,R1,A2,R2,alpha,trigger,L1sp,L2sp,usp);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(uini)
%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 12;
%x(1) Umpc after network
%x(2) Counter
%x(3) Ideal Umpc
%x(4) One time step delay of x(1), compare with hybrid system
%x(5) U trigger
%x(6) two-step trigger
%x(7) xo1
%x(8) xo2
%x(9) uO
%x(10) xc1
%x(11) xc2
%x(12) uC
sizes.NumOutputs     = 10;
sizes.NumInputs      = 4;   % L1, L2, LR, send2
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%

x0  = [uini,1,uini,uini,30,0,9,10,uini,6,7,uini];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,delta_t,delta_tc,A1,R1,A2,R2,alpha,trigger,L1sp,L2sp,usp)
% global current2 future2
x(4)=x(1);
if (rem(x(2),delta_tc/delta_t)==0)
if (x(2)<=3*delta_tc/delta_t)||(x(2)==round(u(4)))    
    L1=u(1);
    L2=u(2);
    LR=u(3);   
    %common constant
    rho=1000;       %density of water 1kg/m3
    g=9.8;          %gravity constant, m/s
    %parameters of BASIN
    AR=1;           %Area of basin,m2
    A=1/(A1*rho);
    B=1/(A2*rho);
    C=1/(AR*rho);
    D=sqrt(rho*g)/(R1*rho);
    E=sqrt(rho*g)/(R2*rho);
    SSMA=[1-delta_tc*A*D*1/(sqrt(L1)) 0                            0;
          delta_tc*B*D*1/(sqrt(L1))   1-delta_tc*B*E*1/(sqrt(L2)) 0;
          0                            delta_tc*C*E*1/(sqrt(L2))   1];

    SSMB=[delta_tc*A*alpha 0 -C*alpha*delta_tc]';
    
    %state feedback controller
    K = [-60,-80];
    uu = K*[(L1-L1sp);(L2-L2sp)] -usp;
    if uu>=200
        uu=200;
    end 
    if uu<=-200
        uu=-200;
    end

    
    max_inter=2;
    Lp=zeros(max_inter+1,3);
    LpC=zeros(max_inter+1,3);
    LpO=zeros(max_inter+1,3);
    Lp(1,:) = [u(1), u(2), u(3)];   
    for j=1:max_inter
        Lp(j+1,:)= SSMA*Lp(j,:)'+SSMB*uu; 
    end
    
    for i=2:max_inter
        if abs(Lp(i,2)-Lp(1,2))>=trigger
            break;
        end
    end
    current = x(2);
    future = 2;
        
    uC=K*[(Lp(i,1)-L1sp);(Lp(i,2)-L2sp)]-usp;
    uO=K*[(Lp(1,1)-L1sp);(Lp(1,2)-L2sp)]-usp;
    LpC(1,:)=Lp(i,:);
    LpO(1,:)=Lp(i,:);
    
    % 1-step prediction of LpC and LpO
    for j= 1:future        
        LpC(j+1,:)= SSMA*LpC(j,:)'+SSMB*uC;
    end
        
    for j= 1:future       
        LpO(j+1,:)= SSMA*LpO(j,:)'+SSMB*uO;
    end
    
    
    x(7)= abs(LpO(future,1)-L1sp);
    x(8)= abs(LpO(future,2)-L2sp);
    x(9)=abs(uO - usp);
    x(10)= abs(LpC(future,1)-L1sp);
    x(11)= abs(LpC(future,2)-L2sp);
    x(12)=abs(uC - usp);  
    x(1)=uu;
    x(3)=uu;
    x(5)=current+(future-1)*delta_tc/delta_t;
end  
end
 x(2) = x(2) + 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys = [x];
% end mdlUpdate
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
sys = [x(4);x(5);x(6);x(3);x(7);x(8);x(9);x(10);x(11);x(12)];
% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
