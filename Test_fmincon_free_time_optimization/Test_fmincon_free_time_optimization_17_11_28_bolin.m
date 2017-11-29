% This program is to test Matlab fmincon
% for different cost function, analytical vs. numerical constraints (search),
% and with/without Jacobian input
% with free time optimization
% for a simple ODE problem with constraints
% dot([x;y])=[0 0.310625;0 0]*[x;y]+[0;1]*u
% Input with constraints: u=[0,10]-->[-10,10]
% Initial condition: [x;y]_0=[-1;0]
% Target: [x;y]_10=[1;0] in 10 time steps. 

clear all;
close all; 
clc;

%% System parameters 
sysParam.N = 100; % Time step for free time optimization (number of imputs)
sysParam.delta_ts = 1/sysParam.N;
sysParam.isTargetConstraint = 1; % 1 for target as constraint
sysParam.NumStates = 2; % Number of states
sysParam.zDim_per_step = 1 + sysParam.NumStates + 1; % 1 for tau, 2 for state, 1 for b

sysParam.zDim = sysParam.N + sysParam.zDim_per_step*(sysParam.N+1); % Dimension of states: N for input and 2*(N+1) for states

sysParam.StartPos = [-1;0];
sysParam.TargetPos = [1;0];

sysParam.u_lb = -10;
sysParam.u_ub = 10;

sysParam.track_lb = 1.2;
sysParam.track_rb = 0.8;
sysParam.trackSegNum = 20;

% Track segmentation
sysParam.track_ang = linspace(pi,0,sysParam.trackSegNum);
sysParam.track_bl = [sysParam.track_lb*cos(sysParam.track_ang);sysParam.track_lb*sin(sysParam.track_ang)];
sysParam.track_cline = [1*cos(sysParam.track_ang);1*sin(sysParam.track_ang)];
sysParam.track_br = [sysParam.track_rb*cos(sysParam.track_ang);sysParam.track_rb*sin(sysParam.track_ang)];

%% System characterisitcs
% Objective function
sysParam.TargetPosWeight = 100;
min_obj = @(z) objFun(z,sysParam);

% Initial condition
u = @(t) -0.3175*sin(pi/10*t-pi/2); % Known control input
x = @(t) [1*cos(pi-pi/10*t);1*sin(pi-pi/10*t)]; % Known trajectory
 % Free time transformation  
sysParam.b0 = 10;
tau0 = @(s) sysParam.b0*s;
u_tilde0 = @(s) u(sysParam.b0*s);
x_tilde0 = @(s) x(sysParam.b0*s);

z0_int = [tau0([0:sysParam.N].*sysParam.delta_ts);...
          x_tilde0([0:sysParam.N].*sysParam.delta_ts);...
          sysParam.b0*ones(1,sysParam.N+1)];

z0 = [u_tilde0((0:sysParam.N-1).*sysParam.delta_ts)';reshape(z0_int,[sysParam.zDim_per_step*(sysParam.N+1),1])] + 0.5*rand(sysParam.zDim,1);

% Constraints
sysParam.isContinuousConstraint = 1; % 1 for circle constraint; 0 for discretized constraint
if sysParam.isContinuousConstraint == 0
    nonlcon = @(z) discretizedCon(z,sysParam);
elseif sysParam.isContinuousConstraint == 1
    nonlcon = @(z) circleCon(z,sysParam);
end

%% fmincon
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
% options = optimoptions('fmincon','Display','iter','Algorithm','sqp' ,'SpecifyConstraintGradient',true, 'SpecifyObjectiveGradient',true, 'MaxIter', 10000, 'MaxFunEvals', 10000);
options = optimoptions(@fmincon,'Display','iter','Algorithm','sqp' ,'SpecifyObjectiveGradient',false, 'SpecifyConstraintGradient',false);
z_sol = fmincon(min_obj,z0,A,b,Aeq,beq,lb,ub,nonlcon,options)

u_sol = z_sol(1:sysParam.N,1);
traj_x_sol = z_sol(sysParam.N+2:sysParam.zDim_per_step:end,1);
traj_y_sol = z_sol(sysParam.N+3:sysParam.zDim_per_step:end,1);
b_sol = z_sol(sysParam.N+sysParam.zDim_per_step:sysParam.zDim_per_step:end,1);

%% Plot
figure(1)
% Plot the whole track
hold on;axis equal;box on;
plot(sysParam.track_bl(1,:),sysParam.track_bl(2,:),'k','lineWidth',1.5);
plot(sysParam.track_br(1,:),sysParam.track_br(2,:),'k','lineWidth',1.5);
plot(sysParam.track_cline(1,:),sysParam.track_cline(2,:),'k--','lineWidth',1);
plot(traj_x_sol,traj_y_sol,'b','lineWidth',2);