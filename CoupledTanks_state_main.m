
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [CONFIDENTIAL]
% Wireless control simulation
% Different scheduling protocol: periodic, OPT scheduling, OPT scheduling +
% sorting
% Objective: Implement simulator of wireless control system, compare OPT
% scheduling algorithm with periodic scheduling
% Transimission time: 8.3 ms
% Control rate: 24 Hz
% Refer to https://github.com/WU-CPSL/WCPSv3_docker
% The multi-rate implementation refers to WCPSv4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
warning off
clear;
clc;
close all;
noise_change = 0;
close_system('wireless_networked_control17')

%% Selection network scheduling algorithm

% 1 periodic scheduling
% 2 OPT scheduling
% 3 OPT scheduling + sorting algorithm
scheduling_algorithm = 2;


%% Experiment parameters
round_count=0;%49;       % round number of simulations -1, ICCPS simulation:49;             
Tf =12; 
system_rate= 960;    % simulation rate of plants: 960 Hz
sensing_rate=24;     % control rate of plant 1 and plant 3: 24 Hz
sensing_rate2=24;    % control rate of plant 2 and plant 4: 24 Hz
delta_tr=8/960;      % Transmission time, time slot length

TfR = Tf;            %simulation TfR
delta_t=1/system_rate; %control period, i.e., 500ms in this case
delta_tc=1/sensing_rate;
delta_tc2=1/sensing_rate2;


%% Controller parameters for loop1 and loop3
uini=20;
trigger=0.0008;
L1sp=8.4375;         %reference water level of tank 1
L2sp=15;             %reference water level of tank 2
usp=-47.92572;       %steady-state control input


%% Controller parameters for loop2 and loop4
uini2=20;
trigger2=0.0008;
L1sp2=6.75;          %reference water level of tank 1
L2sp2=12;            %reference water level of tank 2
usp2=-42.866;        %steady-state control input

%% Network parameters                  
loss_ratio=0;        %Packet loss ratio of network 0-1, 0 means no packet loss
% for noise_level = [76 77 78 80 82 84]
for noise_level = 77
Cost1=[];
Cost2=[];
Cost3=[];
Cost4=[];
load(strcat('link_quality_',num2str(noise_level),'.mat'))
%generate random traces with fix packet delivery ratios
[trace0,trace1,trace2,trace3,trace4]=TraceGeneration(Tf*system_rate*round_count+50000,loss_ratio);%TraceGeneration(trace lengh, loss ratio)

%% Water tank parameters
% water tank parameters of loop1 and loop3
A1=0.01;                        %Area of tank 1
R1=0.0006;                      %resistance of pipe of tank 1                       
A2=0.006;                       %Area of tank 2
R2=0.0008;                      %resistance of the pipe 2
AR=1;                           %Area of basin,m2
% water tank parameters of loop2 and loop4
A3=0.12;                        %Area of tank 1
R3=0.0006;                      %resistance of pipe of tank 1
A4=0.007;                       %Area of tank 2
R4=0.0008;                      %resistance of the pipe 2
AR1=1;                          %Area of basin
alpha1=10;                      %pump motor constant
% water tank parameters of all loops
L1ini=6;                        %initial value of L1
L2ini=7;                        %initial value of L2
LRini=8;                        %initial value of L3
Lub=[50,50,50];
Llb=[0,0,0];
% fixed water tank parameters
rho=1000;                       %density of water kg/m3
g=9.8;                          %gravity constant, m/s
p12=1;                          %pressure at the end of the pipe without tap
R1tap=0.001;                    %resistance of the pipe with tap
p2tap=1;                        %pressure at the end of the pipe with tap
p22=1;                          %pressure at the end of the pipe 2

global solution_set
solution_set = [];


global z1os z1cs z2os z2cs z3os z3cs z4os z4cs prrs

z1os=[];
z1cs=[]; 
z2os=[]; 
z2cs=[]; 
z3os=[];
z3cs=[];
z4os=[]; 
z4cs=[];
prrs = [];
%% RUN Wireless  Process Control Simulation
for yh=0:round_count
global Atrace
noise_level
yh
%% load TOSSIM link quality traces (constant noise level)
trace_count = Tf*sensing_rate*3+1;
Atrace=[link5f(yh*trace_count+1:(yh+1)*trace_count+200),...
    link2f(yh*trace_count+1:(yh+1)*trace_count+200),...
    link3f(yh*trace_count+1:(yh+1)*trace_count+200),...
    link8f(yh*trace_count+1:(yh+1)*trace_count+200)];

%% EKF parameters for loop1
global jX bX bP jP bZ Kk
global deadline1_L deadline2_L Mtime deadline1 deadline2
jX=[5;5;5];
bX=[0;0;0];
bP=eye(3);
jP=eye(3);
bZ=0;
Kk=[0;0;0];
LiniEKF=[9,10,12];
deadline1_L=0;
deadline2_L=0;
deadline1=0;
deadline2=0;
Mtime=0;


%% EKF parameters for loop2
global jX2 bX2 bP2 jP2 bZ2 Kk2
jX2=[5;5;5];
bX2=[0;0;0];
bP2=eye(3);
jP2=eye(3);
bZ2=0;
Kk2=[0;0;0];
LiniEKF2=[6,7,8];

%% EKF parameters for loop3
global jX3 bX3 bP3 jP3 bZ3 Kk3
global deadline3_L deadline4_L deadline3 deadline4 ttt
jX3=[5;5;5];
bX3=[0;0;0];
bP3=eye(3);
jP3=eye(3);
bZ3=0;
Kk3=[0;0;0];
LiniEKF3=[9,10,12];
deadline3_L=0;
deadline4_L=0;
deadline3=0;
deadline4=0;
ttt=0;

%% EKF parameters for loop4
global jX4 bX4 bP4 jP4 bZ4 Kk4
jX4=[5;5;5];
bX4=[0;0;0];
bP4=eye(3);
jP4=eye(3);
bZ4=0;
Kk4=[0;0;0];
LiniEKF4=[6,7,8];

%% Schedule
global schedule
schedule = zeros(Tf/delta_tc+20,5);
%% Trace driven simulation
global SS1 TT1 SS2 TT2 SS3 TT3 SS4 TT4 %link quality prediction algorithm variables
SS1 = 1;
SS2 = 1;
SS3 = 1;
SS4 = 1;
TT1 = 0;
TT2 = 0;
TT3 = 0;
TT4 = 0;
%% debug link quality prediction
global PRR1 PRR2 PRR3 PRR4 PRR_pre1 PRR_pre2 PRR_pre3 PRR_pre4
PRR1 = [];
PRR2 = [];
PRR3 = [];
PRR4 = [];
PRR_pre1 = [];
PRR_pre2 = [];
PRR_pre3 = [];
PRR_pre4 = [];
%%


TraceS=trace1(yh*(Tf*system_rate+1)+1:(yh+1)*(Tf*system_rate+1)+1);  % Sensing traces

tic       
  option = simset('solver','ode4','FixedStep',delta_t);

    sim('wireless_networked_control_HIL.mdl') 
toc

%% Plot results
plot_control_results;
plot_network_results;
plot_network_zoomin;
metric_calculate;
% save(strcat('data_time_lyap_net_pri_24Hz_long_L1L2_76_sub_',num2str(yh)));
end

% save(strcat('data_time_lyap_net_24Hz_long_L1L2_',num2str(noise_level)));
% save(strcat('data_time_lyap_net_pri_24Hz_long_L1L2_3_',num2str(noise_level)));
% save('solution_set_error');
end