%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [CONFIDENTIAL]
% S-function of network manager
% Frist, draft schedule (draft_schedule function)
% Second, 3 scheduling protocols: periodic, OPT scheduling, OPT scheduling
% + sorting
% Third, PRR prediction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,x0,str,ts,simStateCompliance] = Networksfun(t,x,u,flag,delta_t,delta_tr,delta_tc,delta_tc2,scheduling_algorithm)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u);

  case 1,
    sys=mdlDerivatives(t,x,u);

  case 2,
    sys=mdlUpdate(t,x,u,delta_t,delta_tr,delta_tc,delta_tc2,scheduling_algorithm);

  case 3,
    sys=mdlOutputs(t,x,u);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(t,x,u)


sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 53;
% x(1) 1 means agent 1 is accessing the network
% x(2) 1 means agent 2 is accessing the network
% x(3) the same as u(1), trigger (trigger and predicted time) of loop1
% x(4) the same as u(2), trigger (trigger and predicted time) of loop2
% x(5) last trigger of loop1
% x(6) last trigger of loop2
% x(7) counter
% x(8) send1: the time when agent1 should send control inputs
% x(9) send2: the time when agent2 should send control inputs

% x(10) 1 means agent 3 is accessing the network
% x(11) 1 means agent 4 is accessing the network
% x(12) the same as u(3), trigger (trigger and predicted time) of loop3
% x(13) the same as u(4), trigger (trigger and predicted time) of loop4
% x(14) last trigger of loop3
% x(15) last trigger of loop4
% x(16) send3: the time when agent3 should send control inputs
% x(17) send4: the time when agent4 should send control inputs

% x(18) packet counter of loop1
% x(19) packet counter of loop2
% x(20) packet counter of loop3
% x(21) packet counter of loop4

% x(22) last x(1)
% x(23) last x(2)
% x(24) last x(10)
% x(25) last x(11)

% x(26) predicted PRR of loop1
% x(27) predicted PRR of loop2
% x(28) predicted PRR of loop3
% x(29) predicted PRR of loop4

% x(30) xo11
% x(31) xo12
% x(32) CostO1
% x(33) xc11
% x(34) xc12
% x(35) CostC1

% x(36) xo21
% x(37) xo22
% x(38) CostO2
% x(39) xc21
% x(40) xc22
% x(41) CostC2

% x(42) xo31
% x(43) xo32
% x(44) CostO3
% x(45) xc31
% x(46) xc32
% x(47) CostC3

% x(48) xo41
% x(49) xo42
% x(50) CostO4
% x(51) xc41
% x(52) xc42
% x(53) CostC4


sizes.NumOutputs     = 8;
% x(1)
% x(2)
% x(8)
% x(9)
% x(10)
% x(11)
% x(16)
% x(17)

sizes.NumInputs      = 34;
% u(1) trigger (trigger and predicted time) of loop1
% u(2) trigger (trigger and predicted time) of loop2
% u(3) trigger (trigger and predicted time) of loop3
% u(4) trigger (trigger and predicted time) of loop4
% u(5) 1 means that loop1 has two trigger
% u(6) 1 means that loop3 has two trigger

% u(7) xo11
% u(8) xo12
% u(9) uO1
% u(10) xc11
% u(11) xc12
% u(12) uC1

% u(13) xo21
% u(14) xo22
% u(15) uO2
% u(16) xc21
% u(17) xc22
% u(18) uC2

% u(19) xo31
% u(20) xo32
% u(21) uO3
% u(22) xc31
% u(23) xc32
% u(24) uC3

% u(25) xo41
% u(26) xo42
% u(27) uO4
% u(28) xc41
% u(29) xc42
% u(30) uC4

%added for HIL simulation
% u(31) uu1
% u(32) uu2
% u(33) uu3
% u(34) uu4

sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0=[1;1;1;1;0;0;1;1;1;1;1;1;1;0;0;1;1;0;0;0;0;1;1;1;1;1;1;1;1;50;50;20;50;50;20;50;50;20;50;50;20;50;50;20;50;50;20;50;50;20;50;50;20];
str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u)

sys = [];

function sys=mdlUpdate(t,x,u,delta_t,delta_tr,delta_tc,delta_tc2,scheduling_algorithm)
global schedule Atrace
global xo11 xo12 uo1 xc11 xc12 uc1 xo21 xo22 uo2 xc21 xc22 uc2 xo31 xo32 uo3 xc31 xc32 uc3 xo41 xo42 uo4 xc41 xc42 uc4
global xo11L xo12L uo1L xc11L xc12L uc1L xo21L xo22L uo2L xc21L xc22L uc2L ...
    xo31L xo32L uo3L xc31L xc32L uc3L xo41L xo42L uo4L xc41L xc42L uc4L
global deadline1 deadline2 deadline1_L deadline2_L T1L T2L C1L C2L C3L C4L CostC1 CostC2 CostC3 CostC4
global deadline3 deadline4 deadline3_L deadline4_L T3L T4L O1L O2L O3L O4L CostO1 CostO2 CostO3 CostO4
global SS1 TT1 SS2 TT2 SS3 TT3 SS4 TT4
global PRR1 PRR2 PRR3 PRR4 PRR_pre1 PRR_pre2 PRR_pre3 PRR_pre4
global solution_set
global z1os z1cs z2os z2cs z3os z3cs z4os z4cs prrs
slot_length = (1/24)/5;

x(5)=x(3);
x(6)=x(4);
x(14)=x(12);
x(15)=x(13);
x(22)=x(1);
x(23)=x(2);
x(24)=x(10);
x(25)=x(11);

if u(1)~=0
    x(3)=u(1);
    x(30)=u(7);
    x(31)=u(8);
    x(32)=u(9);
    x(33)=u(10);
    x(34)=u(11);
    x(35)=u(12);
end
if u(2)~=0
    x(4)=u(2);
    x(36)=u(13);
    x(37)=u(14);
    x(38)=u(15);
    x(39)=u(16);
    x(40)=u(17);
    x(41)=u(18);
end
if u(3)~=0
    x(12)=u(3);
    x(42)=u(19);
    x(43)=u(20);
    x(44)=u(21);
    x(45)=u(22);
    x(46)=u(23);
    x(47)=u(24);
end
if u(4)~=0
    x(13)=u(4);
    x(48)=u(25);
    x(49)=u(26);
    x(50)=u(27);
    x(51)=u(28);
    x(52)=u(29);
    x(53)=u(30);
end


if deadline1~=x(3)
    T1L=deadline1;
    xo11L=xo11;
    xo12L=xo12;
    uo1L=uo1;
    O1L=CostO1;
    xc11L=xc11;
    xc12L=xc12;
    uc1L=uc1;
    C1L=CostC1;
end

if deadline2~=x(4)
    T2L=deadline2;
    xo21L=xo21;
    xo22L=xo22;
    uo2L=uo1;
    O2L=CostO2;
    xc21L=xc21;
    xc22L=xc22;
    uc2L=uc2;
    C2L=CostC2;
end

if deadline3~=round(x(12))
    T3L=deadline3;
    xo31L=xo31;
    xo32L=xo32;
    uo3L=uo3;
    O3L=CostO3;
    xc31L=xc31;
    xc32L=xc32;
    uc3L=uc3;
    C3L=CostC3;
end

if deadline4~=round(x(13))
    T4L=deadline4;
    xo41L=xo41;
    xo42L=xo42;
    uo4L=uo4;
    O4L=CostO4;
    xc41L=xc41;
    xc42L=xc42;
    uc4L=uc4;
    C4L=CostC4;
end

deadline1=ceil(x(3)*delta_t/slot_length-exp(-6));% this is the target slot number *slot_length/delta_t;
deadline2=ceil(x(4)*delta_t/slot_length-exp(-6));% this is the target slot number *slot_length/delta_t;
deadline3=ceil(x(12)*delta_t/slot_length-exp(-6));% this is the target slot number *slot_length/delta_t;
deadline4=ceil(x(13)*delta_t/slot_length-exp(-6));% this is the target slot number *slot_length/delta_t;
xo11=x(30);
xo12=x(31);
uo1=x(32);
CostO1=x(30)+x(31);
xc11=x(33);
xc12=x(34);
uc1=x(35);
CostC1=x(33)+x(34);
xo21=x(36);
xo22=x(37);
uo2=x(38);
CostO2=x(36)+x(37);
xc21=x(39);
xc22=x(40);
uc2=x(41);
CostC2=x(39)+x(40);
xo31=x(42);
xo32=x(43);
uo3=x(44);
CostO3=x(42)+x(43);
xc31=x(45);
xc32=x(46);
uc3=x(47);
CostC3=x(45)+x(46);
xo41=x(48);
xo42=x(49);
uo4=x(50);
CostO4=x(48)+x(49);
xc41=x(51);
xc42=x(52);
uc4=x(53);
CostC4=x(51)+x(52);

%% %%%%%%%%%%%%%%% scheduler %%%%%%%%%%%%%%%%%%%%%

if (deadline1~=deadline1_L)
   deadline1_L=deadline1;
   priority1 = 1;
   loop_num1 = 1;
   x(8)= draft_schedule(deadline1,delta_tr,delta_t,slot_length,loop_num1,priority1,u(5),u(6));

end

if (deadline2~=deadline2_L)
   deadline2_L=deadline2;
   priority2 = 2;
   loop_num2 = 2;
   x(9)= draft_schedule(deadline2,delta_tr,delta_t,slot_length,loop_num2,priority2,u(5),u(6));

end

if (deadline3~=deadline3_L)
   deadline3_L=deadline3;
   priority3 = 1;
   loop_num3 = 3;
   x(16)= draft_schedule(deadline3,delta_tr,delta_t,slot_length,loop_num3,priority3,u(5),u(6));

end

if (deadline4~=deadline4_L)
   deadline4_L=deadline4;  
   priority4 = 3;
   loop_num4 = 4;
   x(17)= draft_schedule(deadline4,delta_tr,delta_t,slot_length,loop_num4,priority4,u(5),u(6));

end

current_period = floor((x(7))*24/960)+1;
current_slot = floor((x(7)/960)/slot_length-(current_period-1)*5+exp(-6))+1;

if floor((x(7))*24/960)+1~=floor((x(7)-1)*24/960)+1
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%% Periodic TDMA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
schedule(current_period,:)= [0 1 2 3 4]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% use optimization algorithm to solve Txi
if scheduling_algorithm == 2||scheduling_algorithm == 3
    % Weight matrix
    w1=1;
    w2=6;
    w4=4;
    w3=1.2;

    cost_matrix=[O1L,C1L;w2*O2L,w2*C2L;O3L,C3L;w4*O4L,w4*C4L]; % cost matrix
    z1o = w1*[xo11L xo12L*0.1 uo1L*0.0001];
    z1c = w1*[xc11L xc12L*0.1 uc1L*0.0001];
    z2o = w2*[xo21L xo22L*0.1 uo2L*0.0001];
    z2c = w2*[xc21L xc22L*0.1 uc2L*0.0001];
    z3o = w3*[xo31L xo32L*0.1 uo3L*0.0001];
    z3c = w3*[xc31L xc32L*0.1 uc3L*0.0001];
    z4o = w4*[xo41L xo42L*0.1 uo4L*0.0001];
    z4c = w4*[xc41L xc42L*0.1 uc4L*0.0001];

    prr_matrix = [x(26);x(27);x(28);x(29)];
    prrs = [prrs;prr_matrix'];

    prr1 = x(26);
    prr2 = x(27);
    prr3 = x(28);
    prr4 = x(29);
    %% %%%%%%%%%%%%%%%%%%%%% brute force %%%%%%%%%%%%%%%%%%%%%%%%
     solution = [100000 0 0 0 0];
%     for Tx1=0:4
%             for Tx2=0:4-Tx1
%                 for Tx3=0:4-Tx1-Tx2
%                     for Tx4=0:4-Tx1-Tx2-Tx3
%                         Z=[(1-prr1)^Tx1*z1o'+(1-(1-prr1)^Tx1)*z1c';(1-prr2)^Tx2*z2o'+(1-(1-prr2)^Tx2)*z2c';...
%                             (1-prr3)^Tx3*z3o'+(1-(1-prr3)^Tx3)*z3c';(1-prr4)^Tx4*z4o'+(1-(1-prr4)^Tx4)*z4c'];    
% %                         Cost = Z'*Z;  
%                         Cost = (1-prr1)^Tx1*z1o*z1o'+(1-(1-prr1)^Tx1)*z1c*z1c'...
%                             + (1-prr2)^Tx2*z2o*z2o'+(1-(1-prr2)^Tx2)*z2c*z2c'...
%                             + (1-prr3)^Tx3*z3o*z3o'+(1-(1-prr3)^Tx3)*z3c*z3c'...
%                             + (1-prr4)^Tx4*z4o*z4o'+(1-(1-prr4)^Tx4)*z4c*z4c';
%                             if Cost<solution(1,1)
%                                     solution = [Cost,Tx1,Tx2,Tx3,Tx4];
%                             end
%                     end
%                 end
%             end       
%     end
% 
%     Tx1=solution(2);
%     Tx2=solution(3);
%     Tx3=solution(4);
%     Tx4=solution(5);
%     solution_set = [solution_set;t solution];
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% %%%%%%%%%%%%%%%%%%%%%%% Convex QP relaxation %%%%%%%%
    tic;
    data.n  = 3;                     % number of states
    data.L  = 5;                     % superframe length
    data.N  = 4;                     % number of control loops
    data.P  = eye(data.N*data.n);    % solution to MSS Lyapunov equation
    data.ub = data.L - 1;            % upper bound on transmission exponent, Tx
    
    % Constraint matrices
    a1          = ones(1, data.L);
    a2          = 0:data.ub;
    
    opt.beq     = ones(data.N,1);    % RHS of equality constraints
    opt.bineq   = data.ub;           % RHS of inequality constraints
    opt.Aeq     = []; 
    opt.Aineq   = [];
    opt.S       = [];                % Matrix to retrieve Tx from t_tilde
    for j = 1:data.N
        opt.Aeq     = blkdiag(opt.Aeq, a1); % LHS of equality constraints
        opt.Aineq   = [opt.Aineq, a2];      % LHS of ineq constraints
        opt.S       = blkdiag(opt.S, a2);   
    end
    
    % Bounds on t-tilde search space
    opt.lb = 0*ones(1, data.L*data.N).';    
    opt.ub = 1*ones(1, data.L*data.N).';
    
    % Convexification parameter
    opt.delta = 1e-8;
    z1os = [z1os; z1o]; 
    z2os = [z2os; z2o];
    z3os = [z3os; z3o];
    z4os = [z4os; z4o];
    z1cs = [z1cs; z1c];
    z2cs = [z2cs; z2c];
    z3cs = [z3cs; z3c];
    z4cs = [z4cs; z4c];
    zo = [z1o' z2o' z3o' z4o'];
    zc = [z1c' z2c' z3c' z4c'];
    beta = 1- [x(26) x(27) x(28) x(29)];
    Q = [];
        for j = 1:data.N
            q = zeros(1, data.L);
            for jj = 0:data.ub
                q(:,jj+1) = zc(:,j)'*zc(:,j) + (zo(:,j)'*zo(:,j) - zc(:,j)'*zc(:,j))*beta(j)^(jj);
            end
            Q=[Q,q];
        end
        
        % Solving relaxed QP
        options = optimoptions('linprog', 'Display', 'off');
        soln_qp_rlx = linprog(Q, opt.Aineq, opt.bineq, opt.Aeq, opt.beq, opt.lb, opt.ub, [], options);
        toc;
        % Converting back to Tx
        sol = round(opt.S*soln_qp_rlx);
        Tx1=sol(1);
        Tx2=sol(2);
        Tx3=sol(3);
        Tx4=sol(4);
        solution_set = [solution_set;sol' - solution(2:end)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% %%%% Assign slots when use optimal scheduling, (without sorting) %%%%%%%%%%
if scheduling_algorithm == 2
    schedule(current_period,:)=zeros(1,5);
        if Tx1~=0
            schedule(current_period,2:2+Tx1-1)=1;
        end
        if Tx2~=0
            schedule(current_period,2+Tx1:2+Tx1+Tx2-1)=2;
        end
        if Tx3~=0
            schedule(current_period,2+Tx1+Tx2:2+Tx1+Tx2+Tx3-1)=3;
        end
        if Tx4~=0
            schedule(current_period,2+Tx1+Tx2+Tx3:2+Tx1+Tx2+Tx3+Tx4-1)=4;
        end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%%%%%%%%%% sorting algorithm %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if scheduling_algorithm == 3
    %weight matrix for sorting algorithm
    ww1 = 1.1;
    ww2 = 1;
    ww3 = 1.08;
    ww4 = 1;


    current_cost1 = ww1*abs(xc11L-8.4375);
    current_cost2 = ww2*abs(xc21L-6.75);
    current_cost3 = ww3*abs(xc31L-8.4375);
    current_cost4 = ww4*abs(xc41L-6.75);

    Loop_num = [1 2 3 4];
    Tx_matrix = [Tx1 Tx2 Tx3 Tx4];
    Cost_matrix = [current_cost1 current_cost2 current_cost3 current_cost4];
    priority_matrix = [Loop_num;Tx_matrix;Cost_matrix];
    [~,idx] = sort(priority_matrix(3,:),'descend'); % sort just the first column
    sorted_priority_matrix = priority_matrix(:,idx);   % sort the whole matrix

    schedule(current_period,:)=zeros(1,5);
    slot = 1;
    while(slot<5&&sum(sorted_priority_matrix(2,:))~=0)
    if sorted_priority_matrix(2,1)~=0
        schedule(current_period,2+slot-1:2+slot-1)=sorted_priority_matrix(1,1);
        slot = slot + 1;
        sorted_priority_matrix(2,1)=sorted_priority_matrix(2,1)-1;
    end

    if sorted_priority_matrix(2,2)~=0
        schedule(current_period,2+slot-1:2+slot-1)=sorted_priority_matrix(1,2);
        slot = slot + 1;
        sorted_priority_matrix(2,2)=sorted_priority_matrix(2,2)-1;
    end

    if sorted_priority_matrix(2,3)~=0
        schedule(current_period,2+slot-1:2+slot-1)=sorted_priority_matrix(1,3);
        slot = slot + 1;
        sorted_priority_matrix(2,3) = sorted_priority_matrix(2,3)-1;
    end

    if sorted_priority_matrix(2,4)~=0
        schedule(current_period,2+slot-1:2+slot-1)=sorted_priority_matrix(1,4);
        slot = slot +1;
        sorted_priority_matrix(2,4) = sorted_priority_matrix(2,4)-1;
    end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% added for HIL simulation
% schedule(current_period,:)
% %[u(31) u(32) u(33) u(34)]
% [u(30+schedule(current_period,2)) u(30+schedule(current_period,3)) u(30+schedule(current_period,4)) u(30+schedule(current_period,5))]
%%
% result = SocketHandle(u(31),u(32),u(33),u(34),schedule(current_period,2),schedule(current_period,3),schedule(current_period,4),schedule(current_period,5))
end

   
%% Control loops access wireless network in assigned time slots %%%%%%%%%%%   
if schedule(current_period,current_slot)==1
    x(1)=T1L;
    x(2)=0;
    x(10)=0;
    x(11)=0;
elseif schedule(current_period,current_slot)==2
    x(1)=0;
    x(2)=T2L;
    x(10)=0;
    x(11)=0;
elseif schedule(current_period,current_slot)==3
    x(1)=0;
    x(2)=0;
    x(10)=T3L;
    x(11)=0;
elseif schedule(current_period,current_slot)==4
    x(1)=0;
    x(2)=0;
    x(10)=0;
    x(11)=T4L;
else
    x(1)=0;
    x(2)=0;
    x(10)=0;
    x(11)=0;
end

%% %%%%%%%%%%%%%%%% Link quality prediction%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate PRR
window_size = 15; 
forcast_step = 1;
alpha = 0.9;
gamma = 0.1;
current_abs_slot = floor((x(7)/960)/slot_length);
last_abs_slot = floor(((x(7)-1)/960)/slot_length);
% link 1
if floor((x(7))*24/960)+1~=floor((x(7)-1)*24/960)+1||x(22)~=0&&x(1)==0||(x(1)~=0&&current_abs_slot~=last_abs_slot)
    x(18)=x(18)+1;
    % measure current PRR using sliding window filter
    if x(18) <window_size
        PRR = sum(Atrace(1:window_size,1))/window_size;
    else
        PRR = sum(Atrace(x(18)-window_size+1:x(18),1))/window_size;
    end  
    % predict PRR using Holt's additive model
    S1_last = SS1;
    T1_last = TT1;
    SS1 = alpha* PRR + (1 - alpha)*(S1_last + T1_last);
    TT1 = gamma*(SS1 - S1_last) + (1 - gamma)*T1_last;
    x(26) = SS1 + forcast_step*TT1;
    if x(26)>=1
        x(26) = 1;
    end
    if x(26)<=0
        x(26) = 0;
    end
    PRR1 = [PRR1;PRR,t];
    PRR_pre1 = [PRR_pre1;x(26),t];
end

% link 2
if floor((x(7))*24/960)+1~=floor((x(7)-1)*24/960)+1||x(23)~=0&&x(2)==0||(x(2)~=0&&current_abs_slot~=last_abs_slot)
    x(19)=x(19)+1;
    % measure current PRR using sliding window filter
    if x(19) <window_size
        PRR = sum(Atrace(1:window_size,2))/window_size;
    else
        PRR = sum(Atrace(x(19)-window_size+1:x(19),2))/window_size;
    end   
    % predict PRR using Holt's additive model
    S2_last = SS2;
    T2_last = TT2;
    SS2 = alpha* PRR + (1 - alpha)*(S2_last + T2_last);
    TT2 = gamma*(SS2 - S2_last) + (1 - gamma)*T2_last;
    x(27) = SS2 + forcast_step*TT2;
    if x(27)>=1
        x(27) = 1;
    end
    if x(27)<=0
        x(27) = 0;
    end
    PRR2 = [PRR2;PRR,t];
    PRR_pre2 = [PRR_pre2;x(27),t];
end

% link 3
if floor((x(7))*24/960)+1~=floor((x(7)-1)*24/960)+1||x(24)~=0&&x(10)==0||(x(10)~=0&&current_abs_slot~=last_abs_slot)
    x(20)=x(20)+1;
    % measure current PRR using sliding window filter
    if x(20) <window_size
        PRR = sum(Atrace(1:window_size,3))/window_size;
    else
        PRR = sum(Atrace(x(20)-window_size+1:x(20),3))/window_size;
    end    
    % predict PRR using Holt's additive model
    S3_last = SS3;
    T3_last = TT3;
    SS3 = alpha* PRR + (1 - alpha)*(S3_last + T3_last);
    TT3 = gamma*(SS3 - S3_last) + (1 - gamma)*T3_last;
    x(28) = SS3 + forcast_step*TT3;
    if x(28)>=1
        x(28) = 1;
    end
    if x(28)<=0
        x(28) = 0;
    end
    PRR3 = [PRR3;PRR,t];
    PRR_pre3 = [PRR_pre3;x(28),t];
end

% link 4
if floor((x(7))*24/960)+1~=floor((x(7)-1)*24/960)+1||x(25)~=0&&x(11)==0||(x(11)~=0&&current_abs_slot~=last_abs_slot)
    x(21)=x(21)+1;
    % measure current PRR using sliding window filter
    if x(21) <window_size
        PRR = sum(Atrace(1:window_size,4))/window_size;
    else
        PRR = sum(Atrace(x(21)-window_size+1:x(21),4))/window_size;
    end   
    % predict PRR using Holt's additive model
    S4_last = SS4;
    T4_last = TT4;
    SS4 = alpha* PRR + (1 - alpha)*(S4_last + T4_last);
    TT4 = gamma*(SS4 - S4_last) + (1 - gamma)*T4_last;
    x(29) = SS4 + forcast_step*TT4;
    if x(29)>=1
        x(29) = 1;
    end
    if x(29)<=0
        x(29) = 0;
    end    
    PRR4 = [PRR4;PRR,t];
    PRR_pre4 = [PRR_pre4;x(29),t];
end

x(7)=x(7)+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys = [x];


function sys=mdlOutputs(t,x,u)

sys= [x(1),x(2),x(3),x(4),x(10),x(11),x(12),x(13)];


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;             %Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
