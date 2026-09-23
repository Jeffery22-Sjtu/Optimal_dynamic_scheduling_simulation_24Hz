function [sys,x0,str,ts,simStateCompliance] = Mediasfun(t,x,u,flag,delta_t,delta_tr,delta_tc,delta_tc2,uini,Atrace,noise_change)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation of media access
% Frist, record which loop is accessing the channel
% Second, simulate the transmission latency
% Third, induce TOSSIM PRR trace to simulate real-world packet loss
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u,uini);

  case 1,
    sys=mdlDerivatives(t,x,u);

  case 2,
    sys=mdlUpdate(t,x,u,delta_t,delta_tr,delta_tc,delta_tc2,noise_change);%,Atrace);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u,uini)


sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 39;
%%%%%%%% pure ALOHA %%%%%%%
% x1 network access of loop1
% x2 network access of loop2
% x3 transmission counter of loop1
% x4 transmission counter of loop2
% x5 1 means loop1 receives packet
% x6 1 means loop2 receives packet
% x7 counter
% x8 control inputs after network for loop1
% x9 control inputs after network for loop2
% x10 control counter of loop1 -->control input in network
% x11 control counter of loop2 -->control input in network
% x12 loop1 gets the resource of network
% x13 loop2 gets the resource of network
% x14 u(1)
% x15 u(2)
% x16 control inputs arrive time of loop1
% x17 control inputs arrive time of loop2

% x18 network access of loop3
% x19 transmission counter of loop3
% x20 1 mean loop3 receives packet
% x21 control inputs after network for loop3
% x22 control counter of loop3 -->control input in network
% x23 loop3 gets the resource of network
% x24 u(3)
% x25 last control inputs time of loop3

% x26 network access of loop4
% x27 transmission counter of loop4
% x28 1 mean loop4 receives packet
% x29 control inputs after network for loop4
% x30 control counter of loop4 -->control input in network
% x31 loop4 gets the resource of network
% x32 u(4)
% x33 last control inputs time of loop4

% x34 relative time slot
% x35 last relative time slot

% x36 packet counter of loop1
% x37 packet counter of loop2
% x38 packet counter of loop3
% x39 packet counter of loop4


sizes.NumOutputs     = 12;
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0=[1;1;0;0;1;1;1;uini;uini;uini;uini;1;0;0;0;0;0;1;0;1;uini;uini;0;0;0;1;0;1;uini;uini;0;0;0;5;6;1;1;1;1];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)

sys = [];

function sys=mdlUpdate(t,x,u,delta_t,delta_tr,delta_tc,delta_tc2,noise_change)%,Atrace)
global Atrace Atrace_second_half

if noise_change ==1  
if t>=5
    Atrace = Atrace_second_half;
end  
end

if u(1)~=0
    x(14)=u(1);
    x(10)=u(3);
    x(1)=1;
else
    x(1)=0;
end

if u(2)~=0
    x(15)=u(2);
    x(11)=u(4);
    x(2)=1;
else
    x(2)=0;
end

if u(6)~=0
    x(24)=u(6);
    x(22)=u(5);
    x(18)=1;
else
    x(18)=0;
end

if u(8)~=0
    x(32)=u(8);
    x(30)=u(7);
    x(26)=1;
else
    x(26)=0;
end
%% %%%%%%%% Erlang %%%%%%%%%%%%%%%%%
%%%% Network manager is in charge of scheduling to avoid collision %%%

if x(1)==1%x(1)==1&&x(2)==0&&x(18)==0&&x(26)==0
    x(3)=x(3)+1;
    x(12)=1;
    x(4)=0;
    x(19)=0;
    x(27)=0;
    x(13)=0;
    x(23)=0;
    x(31)=0;    
elseif x(2)==1%x(1)==0&&x(2)==1&&x(18)==0&&x(26)==0
    x(4)=x(4)+1;
    x(13)=1;    
    x(3)=0;   
    x(19)=0;
    x(27)=0;
    x(12)=0;
    x(23)=0;
    x(31)=0;
elseif x(18)==1%x(1)==0&&x(2)==0&&x(18)==1&&x(26)==0
    x(19)=x(19)+1;
    x(23)=1;
    x(3)=0;
    x(4)=0;
    x(27)=0;
    x(12)=0;
    x(13)=0;
    x(31)=0;
    
elseif x(26)==1% x(1)==0&&x(2)==0&&x(18)==0&&x(26)==1
    x(27)=x(27)+1;
    x(31)=1;
    x(3)=0;
    x(4)=0;
    x(19)=0;
    x(12)=0;
    x(13)=0;
    x(23)=0;
else
    x(3)=0;
    x(4)=0;
    x(19)=0;
    x(27)=0;
    x(12)=0;
    x(13)=0;
    x(23)=0;
    x(31)=0;
end

if(x(3)==round(delta_tr/delta_t)-1)
    x(36)=x(36)+1;
    if Atrace(x(36),1)==1 % introduce packet reception traces from TOSSIM
        x(5)=1;
    else
        x(5)=0;
    end
    x(3)=0;
else
    x(5)=0;
end

if(x(4)==round(delta_tr/delta_t)-1)
    x(37)=x(37)+1;
    if Atrace(x(37),2)==1
        x(6)=1;
    else
        x(6)=0;
    end
    x(4)= 0;
else
    x(6)=0;
end

if(x(19)==round(delta_tr/delta_t)-1)
    x(38)=x(38)+1;
    if Atrace(x(38),3)==1
        x(20)=1;
    else
        x(20)=0;
    end
    x(19)=0;
else
    x(20)=0;
end

if(x(27)==round(delta_tr/delta_t)-1)
    x(39)=x(39)+1;
    if Atrace(x(39),4) ==1
        x(28)=1;
    else
        x(28)=0;
    end
    x(27)=0;
else
    x(28)=0;
end


 if x(5)==1
     x(16)=x(14);
     x(8)=x(10);
 end
 
 if x(6)==1
     x(17)=x(15);
     x(9)=x(11);
 end
 
 
 if x(20)==1
     x(25)=x(24);
     x(21)=x(22);
 end
 
 if x(28)==1
     x(33)=x(32);
     x(29)=x(30);
 end

 x(7)=x(7)+1;

%%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys = [x];


function sys=mdlOutputs(t,x,u)

sys = [x(8),x(5),x(6),x(9),x(20),x(21),x(28),x(29), x(12),x(13),x(23),x(31)];

function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;             %Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
