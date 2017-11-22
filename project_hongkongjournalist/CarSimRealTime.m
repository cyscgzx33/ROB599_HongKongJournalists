%% W2017 ME 542 Project: Formula 1 car racing: subfunctions
%   Copyright (C)2017 Chaozhe He. All Rights Reserved.
%   Author: Chaozhe He
%           Department of Mechanical Engineering
%           University of Michigan, Ann Arbor
%           March, 2017
% Version 1.0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% PLEASE DO NOT CHANGE THIS FILE %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This include a real time simulation and visualization of a vehicle motion
% on a 2D track
% The core part is an ODE solver based on classical Rough-Kuta method (RK4) 
% The basic function call is
%   
%     [t,Y,U,Totaltime] = CarSimRealTime(fun,Tspan,Y0,para0,dh,usize,Track,Car,Motionon)
%
%   INPUT
% fun gives the ODE right handside of vehicle model
% fun should be of this form
% 
%          [dY,U]=fun(t,Y,para);
% 
% t is time
% Y is current state value 
% para is the parameters( potentially varying, e.g., the reference according to centerline)
% dY gives the derivative
% 
% U is the implicit values include control and the update of para  
%         The default order is 
%     U1=R,U2=gamma,U3=gammadot,U4=s,U5=sf,U6=sr,U7=n,U8=nf,U9=nr; 
%     you may change fun definition but ONLY ADD Y and U AFTER these default ones.
% 
% Tspan gives the initial and final time
% Y0 gives the initial value of states
% para0 gives the initial para value
% dh define the ODE solver step
% usize tells the size of U
% Track is the structure variable for Track information
% Car is the structure variable for Car parameters
% Motionon the indicator on whether there will be a motion or not.
%          =1 motion on
%           =0 motion off

% Any issues/bug reports,
% please email to hchaozhe@umich.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t,Y,U,Totaltime] = CarSimRealTime(fun,Tspan,Y0,para0,dh,usize,Track,Car,Motionon)
%% ODE variables define 
t=(Tspan(1):dh:Tspan(2))';
Y=zeros(length(t),length(Y0));
Y(1,:)=Y0;
U=t*0*zeros(1,usize);
para=para0;
%% setup figures size
Height=6;
Width=7;
FontSize=16;
showzoom=1;
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', FontSize*showzoom)
set(0,'DefaultTextFontname', 'Times New Roman')
set(0,'DefaultTextFontSize', FontSize*showzoom)
set(0,'defaultlinelinewidth',2*showzoom)
figure(101);
set(gcf,'units','inches');
pos_default = get(gcf,'pos');
pos1=pos_default;
pos1(1)=pos1(1)+pos1(3)/2-Width;
pos1(2)=pos1(2)-(Height-pos1(4));
pos1(3)=Width;
pos1(4)=Height;
pos2=pos_default;
pos2(1)=pos2(1)+pos2(3)/2;
pos2(2)=pos2(2)-(Height-pos2(4));
pos2(3)=Width;
pos2(4)=Height;
close gcf
%%
if Motionon
figure(1)
set(gcf,'units','inches','pos',pos1);
figure(2)
set(gcf,'units','inches','pos',pos2);
end
%% Plot the Track
centerline=Track.cline;
bl=Track.bl;
br=Track.br;
bst=[Track.bstl,Track.bstr];
bf=[Track.bfl,Track.bfr];
bl_c=bl;
br_c=br;
if Motionon
figure(1)
% zoom view
axis equal
hold on
plot3(centerline(1,:),centerline(2,:),centerline(3,:),'k--');
plot3(bl_c(1,:),bl_c(2,:),bl_c(3,:),'k','Markersize',5);
plot3(br_c(1,:),br_c(2,:),br_c(3,:),'k','Markersize',5);
box on;xlabel('x[m]');ylabel('y[m]');
figure(2)
% global view
axis equal
hold on
plot3(centerline(1,:),centerline(2,:),centerline(3,:),'k--');
plot3(bl_c(1,:),bl_c(2,:),bl_c(3,:),'k','Markersize',5);
plot3(br_c(1,:),br_c(2,:),br_c(3,:),'k','Markersize',5);
box on;xlabel('x[m]');ylabel('y[m]');

figure(1)
plot3(bf(1,:),bf(2,:),bf(3,:),'m','LineWidth',10);
plot3(bst(1,:),bst(2,:),bst(3,:),'g','LineWidth',5);
figure(2)
plot3(bf(1,:),bf(2,:),bf(3,:),'m','LineWidth',10);
plot3(bst(1,:),bst(2,:),bst(3,:),'g','LineWidth',5);

%%%%%%%% Initial position
Xcar=Y0;
figure(1)
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
figure(2)
L(2)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
end
%
wheelscale=2;
%
for i=1:length(t)-1
    
%%%%%%%%%%%%%%%%%%%%%% cole part for classical Runge Kutta  %%%%%%%%%    
    [k1,uu]=fun(t(i),Y(i,:)',para);k1=k1';para=uu(4);
    k2=fun(t(i)+dh/2,(Y(i,:)+k1*dh/2)',para)';
    k3=fun(t(i)+dh/2,(Y(i,:)+k2*dh/2)',para)';
    k4=fun(t(i)+dh,(Y(i,:)+k3*dh)',para)';
    Y(i+1,:)=Y(i,:)+dh/6*(k1+2*k2+2*k3+k4);
    U(i,:)=uu';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get the lateral force for constraint check
[Ffl,Frl]=Force_rwd(Y(i,4),U(i,1)-Car.k*Y(i,4)^2,U(i,2),U(i,3),Car.m,Car.m0,Car.b,Car.w);
% get the distance to the boundary for on/off track detection
x=Y(i,1);y=Y(i,2);psi=Y(i,3);sigma=Y(i,4);gamma=U(i,2);
pfront=Car.a*[cos(psi);sin(psi);0]+[x;y;0];
prear=-Car.b*[cos(psi);sin(psi);0]+[x;y;0];
% s0;sf;sr;n0;nf;nr
s0=U(i,4);sf=U(i,5);sr=U(i,6);
n0=U(i,7);nf=U(i,8);nr=U(i,9);
%%%%%%%%%%%%%%%%%%%%%%%% Constraint check %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% laterial force 

% boundary
leftedge=[[x;y;0],Track.fun_bl(s0)];
rightedge=[[x;y;0],Track.fun_br(s0)];
leftedgef=[pfront,Track.fun_bl(sf)];
rightedgef=[pfront,Track.fun_br(sf)];
leftedger=[prear,Track.fun_bl(sr)];
rightedger=[prear,Track.fun_br(sr)];
ipc=(leftedge(:,2)-leftedge(:,1))'*(rightedge(:,2)-rightedge(:,1));
ipf=(leftedgef(:,2)-leftedgef(:,1))'*(rightedgef(:,2)-rightedgef(:,1));
ipr=(leftedger(:,2)-leftedger(:,1))'*(rightedger(:,2)-rightedger(:,1));

if Motionon
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot motion  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% car body
Xcar=[x;y;0];
tv=[cos(psi);sin(psi);0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];
% front wheel angle

Rotate=[cos(psi+gamma) -sin(psi+gamma)  0
    sin(psi+gamma)  cos(psi+gamma)  0
         0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
figure(1)
delete(L);
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',3);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',5);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',5);
L(5)=plot3(leftedge(1,:),leftedge(2,:),leftedge(3,:),'c--','LineWidth',2);
L(6)=plot3(rightedge(1,:),rightedge(2,:),rightedge(3,:),'c--','LineWidth',2);
L(7)=plot3(leftedgef(1,:),leftedgef(2,:),leftedgef(3,:),'c--','LineWidth',2);
L(8)=plot3(rightedgef(1,:),rightedgef(2,:),rightedgef(3,:),'c--','LineWidth',2);
L(9)=plot3(leftedger(1,:),leftedger(2,:),leftedger(3,:),'c--','LineWidth',2);
L(10)=plot3(rightedger(1,:),rightedger(2,:),rightedger(3,:),'c--','LineWidth',2);
xlim([Xcar(1)-10,Xcar(1)+10]);
ylim([Xcar(2)-10,Xcar(2)+10]);
legend(L(1),sprintf('Time:%.2f\n Speed: %.2f\n Front %.1f[N] \n Rear %.1f[N]',t(i),sigma,Ffl,Frl),'Location','Southeast');
drawnow;
figure(2)
L(11)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
drawnow;
else
% Show data in workspace only 
sigma=Y(i,4);
clc;fprintf('Time:%.2f\n Speed: %.2f\n Front %.1f[N] \n Rear %.1f[N]',t(i),sigma,Ffl,Frl);    
end
if ipc>0||ipf>0||ipr>0
   error('You are offtrack!');
end
if abs(Ffl)>Car.Ffl_max||abs(Frl)>Car.Frl_max
    if abs(Ffl)>Car.Ffl_max
   error('Insufficient lateral force at front wheel!'); 
    end
    if abs(Frl)>Car.Frl_max
   error('Insufficient lateral force at rear wheel!');      
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%% Terminal Condition  %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% when the center of mass crosse the finish line %%%%%%%%%%%%%%%%
    if i>1&&lineintersect(Y(i,1:2),Y(i+1,1:2),bf(1:2,1),bf(1:2,2))
       U=U(1:i+1,:); U(i+1,:)=U(i,:);
       t=t(1:i+1);Y=Y(1:i+1,:);
       if Motionon
 % final plot  
 
 x=Y(i+1,1);y=Y(i+1,2);psi=Y(i+1,3);gamma=U(i+1,2);
pfront=Car.a*[cos(psi);sin(psi);0]+[x;y;0];
prear=-Car.b*[cos(psi);sin(psi);0]+[x;y;0];
% car body
Xcar=[x;y;0];
tv=[cos(psi);sin(psi);0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];
% front wheel angle
Rotate=[cos(psi+gamma) -sin(psi+gamma)  0
    sin(psi+gamma)  cos(psi+gamma)  0
         0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
figure(1)
delete(L);
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',3);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',5);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',5);
Totaltime=t(i)+sqrt(sum((Y(i,1:2)-Y(i+1,1:2)).^2))/(Y(i,4)+Y(i+1,4))*2;
legend(L(1),sprintf('TotalTime:%.4f',Totaltime),'Location','Southeast'); 
figure(2)
plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
fprintf('TotalTime:%.4f\n',Totaltime);
drawnow;
        end
       return 
    end
end
end


% Thif function test whether two line intersect with eachother
function [intersect,x,y]=lineintersect(X1,X2,Y1,Y2)
a1=X2(2)-X1(2);
b1=-X2(1)+X1(1);
c1=a1*X1(1)+b1*X1(2);
a2=Y2(2)-Y1(2);
b2=-Y2(1)+Y1(1);
c2=a2*Y1(1)+b2*Y1(2);
det=a1*b2-a2*b1;
if det
   x=(b2*c1-b1*c2)/det;
   y=(a1*c2-a2*c1)/det;
else
    intersect=0;
    x=NaN;y=NaN;
    return;
end
if x<=max(Y1(1),Y2(1))&&x>=min(Y1(1),Y2(1))&&x<=max(X1(1),X2(1))&&x>=min(X1(1),X2(1))
    intersect=1;
else
    intersect=0;
end
end
