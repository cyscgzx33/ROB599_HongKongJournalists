%% W2017 ME 542 Project: Formula 1 car racing Main script
%   Copyright (C)2017 Chaozhe He. All Rights Reserved.
%   Author: Chaozhe He
%           Department of Mechanical Engineering
%           University of Michigan, Ann Arbor
%           March, 2017
% Version 1.0
% Main script
% Any issues/bug reports,
% please email to hchaozhe@umich.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;clc;close all;
%% Car information
load('F1CarData.mat')
Car=CarParameter;
%% Track Information
load('CircuitOfAmerica.mat');
%% Start point set
s_start=0; % start from the race start and end at the same location after one lap

%%%% If you want to start and end somewhere else in the middle  %%%%%%%%%%%
%%%% (for testing perpuse) uncommon and edit the following %%%%%%%%%%%%%%%%
% start_index=11; % 1 to 597
% end_index=21; % 1 to 597
% % start at a certain location 
% s_start=Track.arc_s(start_index);
% Track.bstl=Track.bl(:,start_index);
% Track.bstr=Track.br(:,start_index);
% Track.bfl=Track.bl(:,end_index);
% Track.bfr=Track.br(:,end_index);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
figure(1)
hold on;axis equal;box on
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k','Markersize',5);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k','Markersize',5);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','Markersize',5);
for i=1:length(Track.bl(1,:))
    if mod(i-1,20)==0
plot3([Track.bl(1,i) Track.br(1,i)],[Track.bl(2,i) Track.br(2,i)],[Track.bl(3,i) Track.br(3,i)],'b','LineWidth',1)
    end

end

Track.arc_s;

%%%%%%%%% initial condition %%%%%%%%%%%%%%%
XX0=Track.center(s_start);
v0=sqrt(Car.R_max/Car.k); % initial longitudinal speed top speed
x0=[XX0(1:2);Track.ftheta(s_start);v0];


%% Present your control design here
%% Open loop control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Driving force
% Driving force 
t1=[0,   2,       2.4,  4.3,    7,  8.85,   8.9,   11     11.9   13     13.1    13.6    13.7   17.7    18  18.8  18.9   20.75  20.8   22   22.1      22.4      22.5   23.1  23.2   23.5   23.6 24.6 24.7    24.9  25.1   26    27    27.1   27.4   27.5 28   28.4   28.45  28.5   30.8  30.9  32.2  32.3  34  35.2   35.3 35.4   35.5   36.5  36.6   36.75  36.8   37.1   37.2  37.9     38    38.3      38.4  39.1      39.3     39.7    39.8     39.9      40     41          41.1    41.2                        49.9           50       52.3    52.6   52.9  53.4  53.5   53.8  53.9  71.5  71.6    72.1   74.9   75   79.6   79.7   79.8   81.5   81.6   84  84.1    84.5  84.6  87.8  87.9  89.0   89.1  89.8  89.9   91.0  91.1  96    96.1   96.8  96.9 99.7  99.8   100.7 100.8   106.85  106.9  108.55  108.6  108.7  108.8 109.5 109.7 109.8 109.9 110 110.6  111  114.8  114.85  116.5  116.55 116.85 116.9  117.15  117.2 117.5 118  118.5 130];
R=[5500, 5500, -10000, -10000, 2800, 3800,  5500,  5500    5500  5500   -10000  -10000  5500  5500    5500 5500 -10000 -10000  5500  5500  -10000   -10000     5500    5500 -10000 -10000 5500 5500 -10000 -10000 5500 5500    5500 -10000 -10000  5500 5500 -10000 -10000  5500 5500 -10000 -10000 5500 5500 5500   5500 5500 -10000   -10000 5500   5500  -10000  -10000 5500 5500    -10000 -10000     5500  5500   -10000   -10000    5500    -10000     5500  5500        -8000    5500                        5500         -10000    -10000 -10000   5500  5500 -10000 -10000 5500  5500 -10000 -10000 -10000  5500 5500 -10000 -10000 -10000  5500   5500  -10000 -10000 5500  5500 -10000 -10000 5500  5500 -10000 -10000 5500  5500 -10000 -10000 5500 5500 -10000 -10000 5500    5500   -10000 -10000     500  -2500  -2500 -600      0  500   500  500 1700   5500  5500 -10000  -10000    600   300  -10000  -10000  2000  2000  1000 5500  5500];
Rcontrol=@(t) interp1(t1,R,t);
 
% steering angle
t2=[ -0.1,  4.1,  5.5,  6.5,   7,   8.85,    10.18,   10.3,       11,     12.5,    13,    13.5,     13.9      16    16.5  17    17.8    18.8   19.5      20    20.3   20.8    21    21.5    21.85   22    22.2   22.4    22.7   23.8   24.3    24.6   24.7    25.5     26    27     28    29     30     30.8    31.2  31.5  32.1  32.5   33    34     34.9      36    36.3   36.5   36.7    36.88   37.09   37.4   37.53    37.62  37.95   38.3   39    39.2   39.5  39.85  39.9  40   40.2 40.5 40.8   41.05  41.15 41.3  41.6  41.8   41.9   42.5 42.6 43.8  45   46    46.4   47     48.5    49 51.4   51.65  51.9   52    52.1  52.2   52.3  52.4 52.5   52.6  52.7 52.8   53   53.1  53.2  53.4  53.6   53.8   53.85   53.9    54   54.1    54.2   54.3   54.4   54.5   54.6   54.7  54.8  55       56       57     58      60     63        69       70       71     71.9     72.9    73.4   74    74.5    74.6    74.8  74.9    75.1   75.3   75.5  76   76.5   77     77.2  79      80    80.5    80.7     81.2   81.6   81.9    82    83      84     84.5   84.6     85     85.6   87    88     89   89.9   90.4  90.6  90.9  91.1  91.6   92    93    94    95      96       96.6   97     97.5    98      99    100   100.8   101       102   102.5     104     106    107     108     108.5   108.8   109.5 109.7 109.8 109.9 110    110.5 110.6 110.7 111   112.5  112.55  115.5  116   116.5  117   117.25  118   118.5 119   120  121  122   128];
gamm=[0,     0,   0.09, 0.1,  0.12, 0.06    -0.015,  -0.021,    -0.022, -0.019,   -0.017,  -0.021  -0.026  -0.017    0     0.015 0.014   0.0125  0.0125 -0.025 -0.035 -0.06   -0.055 -0.024  0.027  0.036  0.038  0.05   0.042   0.04   0.02   0.015   -0.01   -0.04 -0.035 -0.027 -0.032 -0.03  -0.025  0.005  0.024  0.03  0.06  0.065  0.052 0.033 -0.018   -0.03  -0.048 -0.061 -0.058  -0.061  -0.083  -0.08  -0.075   -0.07   -0.06  -0.077 -0.065 -0.05  0.07  0.11   0.11  0.121 0.1 0.09 0.077  0.067  0.07  0.065  0.05  0.019  0.001  0  -0.001 0   0.02  0.017 0.016  0.001  -0.001  0   0     0.038 0.05  0.056   0.063 0.072 0.082 0.093 0.105  0.12 0.138 0.148 0.136 0.125 0.114  0.1  0.102   0.152  0.165   0.165  0.159  0.149  0.137 0.127  0.117   0.11  0.102  0.095 0.09  0.07    -0.001 -0.0013 -0.0013 -0.0013 -0.0012   -0.0001 -0.00135 -0.0016 -0.0018   0.0075  0.025 0.0405 0.075   0.09    0.11  0.156  0.174  0.148  0.13 0.098  0.072  0.045  0     0.0025 -0.005   -0.045  -0.06  -0.1  -0.165 -0.134 -0.115 -0.07  -0.045 -0.055   -0.07   -0.06 -0.048   0  0.024  0.055   0.047  0.082 0.11  0.18  0.234 0.155  0.12  0.05  -0.004 -0.003 -0.008  -0.03  -0.038  -0.03  -0.028 -0.02  -0.021 -0.045  -0.028  -0.03 -0.0274  -0.0175  -0.0108  0.003  0.01    0.045   0.054   0.063 0.064 0.065 0.066 0.064  0.055 0.053 0.02  0.007 0.0018      0    0    0.05   0.07  0.095  0.135  0.125  0.08 0.015 0.01 -0.0013 0    0];
gamma=@(t) interp1(t2,gamm,t);

% gammadot=@(t) 0*t;
% % % or rely on numerical estimation
dt=1e-6;
diff_num=@(f,dt,t) (f(t+dt)-f(t-dt))/(2*dt);
gammadot=@(t) diff_num(gamma,dt,t); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% make sure you set a time long enough %%%%%%%%%%%%%%%%%%%

Time=200;
sim_step=0.05;
t_plot=0:sim_step:10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Check for the input constraints
[Rcontrol_real,gamma_real,gammadot_real]=InputChecker(Rcontrol,gamma,gammadot,Car,Time,sim_step);
%%
usize=9;
%%%  By default, the car RWD will give out dX and 9 values for monitoring purpose
%%% They are [R;tgamma;dtgamma;s0;sf;sr;n0;nf;nr]; at time t
% R driving force. tan(gamma), d(tan(gamma))/dt,
% s0 reference to the centerline of the center of mass,
% sf reference to the centerline of the front axle 
% sr reference to the centerline of the rear axle
% n0 distance to the centerline from the center of mass,
% nf distance to the centerline from the front axle 
% nr distance to the centerline from the rear axle
% IF you define more value to be monitored in car_RWD, you need to change
% this number accordingly

car_dynamics=@(t,x,y,psi,sigma,s) car_RWD(t,x,y,psi,sigma,gamma_real(t),gammadot_real(t),Rcontrol_real(t),Track,Car,s);                    
sys2=@(t,x,para) car_dynamics(t,x(1),x(2),x(3),x(4),para);

%% Run
Animation=1; % Animation on
% Animation=0; % Animation off
[t2,y2,u2]=CarSimRealTime(sys2,[0 Time],x0,s_start,sim_step,usize,Track,Car,Animation); 

%% Lateral Forces
[Ffl_ana,Frl_ana]=Force_rwd(y2(:,4),u2(:,1),u2(:,2),u2(:,3),Car.m,Car.m0,Car.b,Car.w);   

%% Show the trajectories
L=[];
Height=8;
Width=5;
FontSize=16;
showzoom=1;
figure(101);
set(gcf,'units','inches');
pos_default = get(gcf,'pos');
pos1=pos_default;
pos1(1)=pos1(1)+(pos1(3)-Width)/2;
pos1(2)=pos1(2)-(Height-pos1(4));
pos1(3)=Width;
pos1(4)=Height;
figure(101);
set(gcf,'pos',pos1)
subplot(4,1,1)
plot(t2,y2(:,1));L=[L,ylabel('$x$[m]')];
subplot(4,1,2)
plot(t2,y2(:,2));L=[L,ylabel('$y$[m]')];
subplot(4,1,3)
plot(t2,y2(:,3));L=[L,ylabel('$\psi$[rad]')];
subplot(4,1,4)
hold on;box on;
plot(t2,y2(:,4));L=[L,ylabel('$v$[m/s]')];L=[L,xlabel('$t$[sec]')];
RR=u2(:,1);
sss=u2(:,4);
nnn=u2(:,7);
delta2=u2(:,2);
figure(102)
subplot(4,1,1)
plot(t2,RR);L=[L,ylabel('$R$[N]')];
subplot(4,1,2)
plot(t2,sss);L=[L,ylabel('$s$[m]')];
subplot(4,1,3);hold on
plot(t2,nnn);L=[L,ylabel('$n$[m]')];
subplot(4,1,4)
plot(t2,delta2);L=[L,ylabel('$\delta$[rad]')];L=[L,xlabel('$t$[sec]')];
figure(103)
subplot(2,1,1);
plot(t2,Ffl_ana); L=[L,ylabel('$F_{\rm F}$[N]')];
subplot(2,1,2);
plot(t2,Frl_ana);L=[L,ylabel('$F_{\rm R}$[N]')];L=[L,xlabel('$t$[sec]')];
%% Show route togather with track
figure(3)
hold on;box on;axis equal;
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k-','LineWidth',1);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k-','LineWidth',1);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','LineWidth',1);
plot(y2(:,1),y2(:,2),'LineWidth',2);
set(L,'Interpreter','latex');