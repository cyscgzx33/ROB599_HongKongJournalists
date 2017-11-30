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

%% Part of the track and feasible input
load('TestTrack');
sysParam.track_bl=TestTrack.bl(:,1:33);
sysParam.track_br=TestTrack.br(:,1:33);
sysParam.track_cline=TestTrack.cline(:,1:33);

% Bo Lin's input
steering_input = [-0*ones(200,1);-0.01*ones(100,1);-0.2*ones(100,1);-0.3*ones(100,1);-0.3*ones(100,1);-0.3*ones(100,1);-0.1*ones(100,1);0*ones(100,1);-0.05*ones(100,1);...
        -0.05*ones(100,1);-0.05*ones(100,1);-0.05*ones(100,1);-0.05*ones(100,1);0.02*ones(100,1);];
force_input = 6000*[ones(200,1);ones(100,1);ones(100,1);ones(100,1);-10/6*ones(100,1);-10/6*ones(100,1);-10/6*ones(100,1);ones(100,1);...
        ones(100,1);ones(100,1);ones(100,1);ones(100,1);ones(100,1);ones(100,1);];

% Junhao Hu's input
% steering_input = [-0.01*ones(250,1); 0*ones(250,1); 0.02*ones(50,1); 0.02*ones(50,1); 0.02*ones(50,1); -0.05*ones(50,1); -0.05*ones(50,1);  -0.05*ones(50,1);...
%   -0.05*ones(50,1); -0.02*ones(50,1); 0*ones(50,1); -0.02*ones(50,1); -0.05*ones(50,1);  -0.05*ones(50,1); -0.02*ones(50,1); 0*ones(50,1); -0.02*ones(50,1);...
%   -0.05*ones(50,1); -0.02*ones(50,1); -0.02*ones(50,1); 0*ones(50,1); 0*ones(50,1); -0.05*ones(50,1); -0.05*ones(50,1); -0.05*ones(50,1); -0.02*ones(50,1);...
%   0*ones(50,1); 0*ones(50,1);  -0.05*ones(50,1); -0.05*ones(50,1);  -0.05*ones(50,1); 0.02*ones(50,1); 0.02*ones(50,1); -0.02*ones(50,1);  0*ones(50,1); ...
%   0.02*ones(50,1); -0.02*ones(50,1); 0*ones(50,1);];
% force_input = 6000*[ones(250,1); 0*ones(250,1); 0*ones(50,1); -10/6*ones(50,1); -10/6*ones(50,1);0*ones(50,1);0*ones(50,1); 0*ones(50,1); ...
%    0*ones(50,1);  0*ones(50,1);  0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1);...
%    0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1); 0*ones(50,1);... 
%    0*ones(50,1); 0*ones(50,1); ones(20,1); ones(30,1);  ones(50,1); ones(50,1); ones(50,1);  ones(50,1);  ones(50,1); ones(50,1); ...
%    ones(50,1); ones(50,1); -10/6*ones(50,1);];

U_manual = [steering_input, force_input]; % Manual input
[Y_manual]=forwardIntegrateControlInput(U_manual); % Trajectory according to manual input (Problem: why is it not one more than input)
Y_manual=[Y_manual;Y_manual(end,:)]; % Append one more row 

NumStep = 20; % Desired step 
delta_t = 0.01;
t_end = size(steering_input,1)*delta_t;
        
%% System parameters 
sysParam.N = NumStep; % Time step for free time optimization (number of imputs)
sysParam.delta_ts = 1/sysParam.N;
sysParam.isTargetConstraint = 0; % 1 for target as constraint
sysParam.NumStates = 6; % Number of states
sysParam.zDim_per_step = 1 + sysParam.NumStates + 1; % 1 for tau, 6 for state, 1 for b

sysParam.zDim = 2*sysParam.N + sysParam.zDim_per_step*(sysParam.N+1); % Dimension of states: 2*N for input and 8*(N+1) for states

sysParam.StartPos = [287;5;-176;0;2;0];
sysParam.TargetPos = sysParam.track_cline(:,end);

sysParam.u_lb = [-0.5;-10000];
sysParam.u_ub = [0.5;6000];

%% System characterisitcs
% Objective function
sysParam.TargetPosWeight = 100;
min_obj = @(z) objFun(z,sysParam);

% Initial condition
% Free time transformation 
u = @(t) [steering_input(round(t/delta_t+1),1) force_input(round(t/delta_t+1),1)]; % Known control input
x = @(t) Y_manual(round(t/delta_t+1),:); % Known trajectory

sysParam.b0 = size(steering_input,1)*delta_t;
tau0 = @(s) sysParam.b0*s;
u_tilde0 = @(s) u(sysParam.b0*s);
x_tilde0 = @(s) x(sysParam.b0*s);

z0_int = [tau0([0:sysParam.N].*sysParam.delta_ts);...
          x_tilde0([0:sysParam.N].*sysParam.delta_ts)';...
          sysParam.b0*ones(1,sysParam.N+1)];

z0 = [reshape(u_tilde0((0:sysParam.N-1).*sysParam.delta_ts)',[2*sysParam.N,1]);reshape(z0_int,[sysParam.zDim_per_step*(sysParam.N+1),1])] + 1*rand(sysParam.zDim,1);

% Constraints
sysParam.isContinuousConstraint = 0; % 1 for circle constraint; 0 for discretized constraint
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
options = optimoptions(@fmincon,'Display','iter','Algorithm','interior-point', 'MaxIter', 10000, 'MaxFunEvals', 10000 ,'SpecifyObjectiveGradient',false, 'SpecifyConstraintGradient',false);
z_sol = fmincon(min_obj,z0,A,b,Aeq,beq,lb,ub,nonlcon,options)

df_sol = z_sol(1:2:2*sysParam.N,1);
Fx_sol = z_sol(2:2:2*sysParam.N,1);
traj_x_sol = z_sol(2*sysParam.N+2:sysParam.zDim_per_step:end,1);
traj_y_sol = z_sol(2*sysParam.N+4:sysParam.zDim_per_step:end,1);
b_sol = z_sol(2*sysParam.N+sysParam.zDim_per_step:sysParam.zDim_per_step:end,1);

%% Plot
figure(1)
% Plot the whole track
hold on;axis equal;box on;
plot(sysParam.track_bl(1,:),sysParam.track_bl(2,:),'k','lineWidth',1.5);
plot(sysParam.track_br(1,:),sysParam.track_br(2,:),'k','lineWidth',1.5);
plot(sysParam.track_cline(1,:),sysParam.track_cline(2,:),'k--','lineWidth',1);
plot(traj_x_sol,traj_y_sol,'b','lineWidth',2);