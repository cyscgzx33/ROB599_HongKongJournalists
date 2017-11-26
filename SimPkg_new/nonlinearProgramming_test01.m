clear all;clc;close all;

%% Note
% (1) delta T = 1/100;
% (2) haven't included the gradient of cost function
% (3) decision variable is padded as [u_tilda(2*steps x 1), tau((steps+1) x 1), x_tilda(6*(steps+1) x 1), b((steps+1) x 1)]
delta_T = 0.01;
steps = 1/delta_T;

% for convenience, write out the start index of each variable
u_start = 1;
tau_start = 2*steps + 1;
x_start = tau_start + steps + 1;
b_start = x_start + 6*(steps+1);
%% Track Information
load('TestTrack');
% calculate arc_s, which is the reference trajectory length
arc_s = zeros(size(TestTrack.cline,2),1);
for i = 1:size(TestTrack.cline,2)-1
    arc_s(i+1) = sqrt((TestTrack.cline(1,i+1)-TestTrack.cline(1,i))^2+(TestTrack.cline(2,i+1)-TestTrack.cline(2,i))^2) + arc_s(i);
end

%% Nonlinear Programming

ultimatePosition = [1471,814.6];
% delta T = 0.01
fun = @(z) z(1008)^2 + (z(902)-ultimatePosition(1))^2 + (z(904)-ultimatePosition(2))^2; % z(902)=x(100), z(904)=y(100)

% Inequality contraints: physical constraints
A = zeros(2016,1008);
A(1:200,1:200) = eye(200);
A(1009:1208,1:200) = -eye(200);
b = zeros(2016,1);
b(1:2:199) = 0.5;
b(2:2:200) = 6000;
b(1009:2:1207) = 0.5;
b(1010:2:1208) = 10000;
% Nonlinear constraints: check if out of boundaries & dynamics equations &
% initial conditions
nonlcon = @BoudaryAndDynamics;

Aeq = [];
beq = [];
lb = [];
ub = [];
% x0 = [0,0];  

function [c,ceq] = BoundaryAndDynamics(z)
%% Boundary
arc_x = zeros(101,1);
arc_x(1) = 2.5; % initial offsite
for i = 1:l00
    arc_x(i+1) = sqrt((z(302+6*i)-z(302+6*(i-1)))^2+(z(304+6*i)-z(304+6*(i-1)))^2) + arc_x(i); % z(302)=x(0), z(304)=y(0)
end
pinpointed_xl = interp1(arc_s,TestTrack.bl(1,:),arc_x);
pinpointed_yl = interp1(arc_s,TestTrack.bl(2,:),arc_x);
pinpointed_xr = interp1(arc_s,TestTrack.br(1,:),arc_x);
pinpointed_yr = interp1(arc_s,TestTrack.br(2,:),arc_x);
% traj.x = z(302:6:902); traj.y = z(304:6:904);
vector_bl = [pinpointed_xl,pinpointed_yl] - [z(302:6:902),z(304:6:904)];
vector_br = [pinpointed_xr,pinpointed_yr] - [z(302:6:902),z(304:6:904)];
% ofst is a safety offset 
ofst = 0;
vec_prod = dot(vector_bl',vector_br');
c = vec_prod + ofst;

%% Dynamics
ceq = [];
for i = 1:steps
    ceq = [ceq;
           z(x_start+6*(i-1):x_start+6*(i-1)+5)+delta_T*vehicleDyn(z(x_start+6*(i-1):x_start+6*(i-1)+5),z(u_start+2*(i-1):u_start+2*(i-1)+5))-z(x_start+6*(i-1):x_start+6*(i-1)+5)]
end
%% Iinitial Conditions



end

function xdot = vehicleDyn(x,u)
steer = u(1);
force = u(2);
%constants
W=13720;
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=2921;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;

% slip angle
a_f= rad2deg(steer-atan2(x(4)+a*x(6),x(2)));
a_r= rad2deg(-atan2((x(4)-b*x(6)),x(2)));

% Nonlinear Tire Dynamics
phi_yf= (1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr= (1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

% Generate lateral forces
F_yf= Dy*sin(Cy*atan(By*phi_yf))+Svy;
F_yr= Dy*sin(Cy*atan(By*phi_yr))+Svy;

%vehicle dynamics
xdot =  [x(2)*cos(x(5))-x(4)*sin(x(5));
         (-f*W+Nw*force-F_yf*sin(steer))/m+x(4)*x(6);
         x(2)*sin(x(5))+x(4)*cos(x(5));
         (F_yf*cos(steer)+F_yr)/m-x(2)*x(6);
         x(6);
         (F_yf*a*cos(steer)-F_yr*b)/Iz];
end