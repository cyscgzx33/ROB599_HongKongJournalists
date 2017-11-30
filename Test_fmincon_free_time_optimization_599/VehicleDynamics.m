function [fx,J_fx]=VehicleDynamics(u,x)
%function [fx,J_fx]=VehicleDynamics(x)
%
%Given input and state, returns the vehicle dynamics and its Jacobian. 
%
% INPUTS:
%   u           an 2-by-1 vector of inputs, where the first row is the
%               steering input in radians, and the second row is the 
%               longitudinal force in Newtons.
%   
%   x           a 6-by-1 vector of the state of the vehicle.
%
% OUTPUTS:
%   fx          a 6-by-1 vector of the vehicle dynamics
%   J_fx        a 8-by-1 vector of the Jacobian of vehicle dynamics

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

%generate input functions
d_f=u(1,1);
F_x=u(2,1);

%slip angle functions in degrees
a_f=@(x) rad2deg(d_f-atan2(x(4)+a*x(6),x(2)));
a_r=@(x) rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=@(x) (1-Ey)*(a_f(x)+Shy)+(Ey/By)*atan(By*(a_f(x)+Shy));
phi_yr=@(x) (1-Ey)*(a_r(x)+Shy)+(Ey/By)*atan(By*(a_r(x)+Shy));


% Generate lateral forces
F_yf=@(x) Dy*sin(Cy*atan(By*phi_yf(x)))+Svy;
F_yr=@(x) Dy*sin(Cy*atan(By*phi_yr(x)))+Svy;

%vehicle dynamics
df=@(x) [x(2)*cos(x(5))-x(4)*sin(x(5));...
        (-f*W+Nw*F_x-F_yf(x)*sin(d_f))/m+x(4)*x(6);...
        x(2)*sin(x(5))+x(4)*cos(x(5));...
        (F_yf(x)*cos(d_f)+F_yr(x))/m-x(2)*x(6);...
        x(6);...
        (F_yf(x)*a*cos(d_f)-F_yr(x)*b)/Iz];

fx = df(x);

%% Jacobian of vehicle dynamics
J_fx= [];

end
