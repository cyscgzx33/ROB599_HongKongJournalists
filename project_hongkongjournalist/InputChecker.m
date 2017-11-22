%% W2017 ME 542 Project: Formula 1 car racing: subfunctions
%   Copyright (C)2017 Chaozhe He. All Rights Reserved.
%   Author: Chaozhe He
%           Department of Mechanical Engineering
%           University of Michigan, Ann Arbor
%           March, 2017
% Version 1.0
% Plot, Check and cap (if needed) the inputs
% Any issues/bug reports,
% please email to hchaozhe@umich.edu
function [Rcontrol_real,gamma_real,gammadot_real]=InputChecker(Rcontrol,gamma,gammadot,Car,Time,sim_step)

gammadata=gamma(0:sim_step:Time);
indexgamma=find(gammadata>Car.gamma_max|gammadata<Car.gamma_min);
Rdata=Rcontrol(0:sim_step:Time);
indexR=find(Rdata>Car.R_max|Rdata<Car.R_min);



%%%%%%%%%%
diff_num=@(f,dt,t) (f(t+dt)-f(t-dt))/(2*dt);dt=1e-6;
if isempty(indexgamma)==0  
   warning('Steering angle input violate constains, cap with bounds and and use numerical gammadot');
   gamma_real=@(t) max(Car.gamma_min,min(Car.gamma_max,gamma(t)));
   gammadot_real=@(t)  diff_num(gamma_real,dt,t);
else   
   gamma_real=@(t) gamma(t);
   gammadot_real=@(t)  gammadot(t);
    
end
if isempty(indexR)==0  
   warning('Driving force input violate constaints, cap with bounds');
   Rcontrol_real=@(t) max(Car.R_min,min(Car.R_max,Rcontrol(t)));
   
else   
   Rcontrol_real=@(t)  Rcontrol(t);
end
t=0:sim_step:100;
L=[];
fig=figure(50);
set(fig,'Name','Your Control Input')
subplot(3,1,1);hold on;box on;
plot(t,Car.R_max+0*t,'k--','LineWidth',2);
plot(t,Car.R_min+0*t,'k--','LineWidth',2);
plot(t,Rcontrol(t),'b','LineWidth',2);
plot(t,Rcontrol_real(t),'r--','LineWidth',2);L=[L,ylabel('$R$[N]')];
subplot(3,1,2);hold on;box on;
plot(t,Car.gamma_max+0*t,'k--','LineWidth',2);
plot(t,Car.gamma_min+0*t,'k--','LineWidth',2);
plot(t,gamma(t),'b','LineWidth',2);
plot(t,gamma_real(t),'r--','LineWidth',2);L=[L,ylabel('$\gamma$[rad]')];
subplot(3,1,3);hold on;box on;
plot(t,gammadot(t),'b','LineWidth',2);
plot(t,gammadot_real(t),'r--','LineWidth',2);L=[L,ylabel('$\dot\gamma$[rad/s]')];
set(L,'Interpreter','latex');
end