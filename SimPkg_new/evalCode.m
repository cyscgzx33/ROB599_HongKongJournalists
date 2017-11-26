%% F2017 ME 599 Project
clear all;clc;close all;
%% Car information
% Car parameters already included in 'forwardIntegrateControllInput.m'
%% Track Information
load('TestTrack');
% calculate arc_s, which is the reference trajectory length
arc_s = zeros(size(TestTrack.cline,2),1);
for i = 1:size(TestTrack.cline,2)-1
    arc_s(i+1) = sqrt((TestTrack.cline(1,i+1)-TestTrack.cline(1,i))^2+(TestTrack.cline(2,i+1)-TestTrack.cline(2,i))^2) + arc_s(i);
end


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
traj = struct; Car = struct;

steer_input = [-0*ones(250,1);-0.0255*ones(80,1);-0.2*ones(70,1);-0.3*ones(150,1);-0.3*ones(200,1);... % 750
               -0.062*ones(100,1);-0.08*ones(250,1);-0.02*ones(200,1)];
% steer_input = [-0.000*ones(300,1);-0.2*ones(300,1);-0.5*ones(150,1)];
force_input = 6000*[ones(300,1);ones(205,1);-10/6*ones(245,1);... % 750
                    ones(550,1)]; 
U = [steer_input, force_input];
[Y]=forwardIntegrateControlInput(U);

% traj = [Y(:,1),Y(:,3)];
traj.x = Y(:,1);
traj.vx = Y(:,2);
traj.y = Y(:,3);
traj.vy = Y(:,4);
traj.psi = Y(:,5);

% derive some interp1 mapping
s = zeros(length(traj.x),1);
s(1) = 2.5; % initial offsite
for i = 1:length(traj.x)-1
    s(i+1) = sqrt((traj.x(i+1)-traj.x(i))^2+(traj.y(i+1)-traj.y(i))^2) + s(i);
end
pinpointed_xl = interp1(arc_s,TestTrack.bl(1,:),s);
pinpointed_yl = interp1(arc_s,TestTrack.bl(2,:),s);
pinpointed_xr = interp1(arc_s,TestTrack.br(1,:),s);
pinpointed_yr = interp1(arc_s,TestTrack.br(2,:),s);
% derive the vectors pointing to pinpointed points of left/right boundaries
vector_bl = [pinpointed_xl,pinpointed_yl] - [traj.x,traj.y];
vector_br = [pinpointed_xr,pinpointed_yr] - [traj.x,traj.y];
% vec_prod = vector_bl.*vector_br;
vec_prod = dot(vector_bl',vector_br');

%% Test checkIfOutTrack
[dot_prod, OutTrackFlag, s_, s_ref] = checkIfOutTrack([traj.x traj.y]);

%% Test fine tuning about the arc length
pinpointed_xl_ = interp1(arc_s,TestTrack.bl(1,:),s_);
pinpointed_yl_ = interp1(arc_s,TestTrack.bl(2,:),s_);
pinpointed_xr_ = interp1(arc_s,TestTrack.br(1,:),s_);
pinpointed_yr_ = interp1(arc_s,TestTrack.br(2,:),s_);

%%
% test the interp1 mapping results
% plot(pinpointed_xl(300),pinpointed_yl(300),'x');hold on;
% plot(traj.x(300),traj.y(300),'o')

input.steer = U(:,1);
input.force = U(:,2);

% % Check Violation
% checkTrajectory(traj,U)

% display velocity
% for textPattern = 1 show vx and vy
% for textPattern = 0 show speed, arc length, time and average speed
textPattern = 1; 
if textPattern == 0
    formatSpec = "v_{lon} = %f \n v_{lat} = %f";
    A1 = traj.vx(end);
    A2 = traj.vy(end);
    str1 = sprintf(formatSpec,A1,A2);
end
if textPattern == 1
    formatSpec = "Speed = %f \n Arc Length = %f \n Time = %f \n Speed_{avg} = %f";
    A1 = sqrt(traj.vx(end)^2+traj.vy(end)^2);
    A2 = s(end);
    A3 = length(traj.x)/100;
    A4 = A2/A3;
    str1 = sprintf(formatSpec,A1,A2,A3,A4);
end


%% Plot

%%% set the plots %%%
Height=6;
Width=7;
FontSize=12;
showzoom=1;
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', FontSize*showzoom)
set(0,'DefaultTextFontname', 'Times New Roman')
set(0,'DefaultTextFontSize', FontSize*showzoom)
% set(0,'defaultlinelinewidth',2*showzoom)
figure(101);
set(gcf,'units','inches');
pos_default = get(gcf,'pos');
pos1=pos_default;
pos1(1)=pos1(1)+pos1(3)/2+Width;
pos1(2)=pos1(2)-(Height-pos1(4));
pos1(3)=Width;
pos1(4)=Height;
pos2=pos_default;
pos2(1)=pos2(1)+pos2(3)/2;
pos2(2)=pos2(2)-(Height-pos2(4));
pos2(3)=Width;
pos2(4)=Height;
pos3=pos_default;
pos3(1)=pos3(1)+pos3(3)/2-Width;
pos3(2)=pos3(2)-(Height-pos3(4));
pos3(3)=Width;
pos3(4)=Height;
close gcf
%% Figure 3
figure(3)
set(gcf,'units','inches','pos',pos3);
figure(1)
set(gcf,'units','inches','pos',pos1);
figure(2)
set(gcf,'units','inches','pos',pos2);

figure(3)
%%%%% plot total track %%%%%
hold on;axis equal;box on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k','lineWidth',1.5);
plot(TestTrack.br(1,:),TestTrack.br(2,:),'k','lineWidth',1.5);
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'k--','lineWidth',1);

%%%%% plot trajectory of mass of center %%%%%
plot(traj.x,traj.y,'r-')

%%%%% plot car body with two tires (end of the moment) %%%%%
wheelscale = 2;
Car.Rw = 0.33;
Car.a=1.35; Car.b=1.45;
Xcar=[traj.x(end);traj.y(end);0];
tv=[cos(traj.psi(end));sin(traj.psi(end));0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];

pfront=Car.a*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];
prear=-Car.b*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];


Rotate=[cos(traj.psi(end)+input.steer(end)) -sin(traj.psi(end)+input.steer(end))  0
        sin(traj.psi(end)+input.steer(end))  cos(traj.psi(end)+input.steer(end))  0
        0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',6);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',2);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',4);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',4);
xlim([200,600])
ylim([-200,200])
% text(traj.x(end)+20,traj.y(end)+5,{str1})

%%%%% plot some marker points %%%%%
plot(traj.x(500),traj.y(500),'rx','LineWidth',2)
text(traj.x(500),traj.y(500),'      \leftarrow This is the 5 second position')
plot(traj.x(750),traj.y(750),'rx','LineWidth',2)
text(traj.x(750),traj.y(750),'      \leftarrow This is the 7.5 second position')


%%%%% plot some arc length to check %%%%%
plot(pinpointed_xr(600),pinpointed_yr(600),'bx','lineWidth',1.5);hold on;
plot(pinpointed_xl(600),pinpointed_yl(600),'bx','lineWidth',1.5);hold on;
plot(traj.x(600),traj.y(600),'ro','lineWidth',1.5); hold on;

plot(pinpointed_xr(300),pinpointed_yr(300),'bx','lineWidth',1.5);hold on;
plot(pinpointed_xl(300),pinpointed_yl(300),'bx','lineWidth',1.5);hold on;
plot(traj.x(300),traj.y(300),'ro','lineWidth',1.5);

plot(pinpointed_xr(1200),pinpointed_yr(1200),'bx','lineWidth',1.5);hold on;
plot(pinpointed_xl(1200),pinpointed_yl(1200),'bx','lineWidth',1.5);hold on;
plot(traj.x(1200),traj.y(1200),'ro','lineWidth',1.5);

%% Figure 4
figure(4)
%%%%% plot total track %%%%%
hold on;axis equal;box on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k','lineWidth',1.5);
plot(TestTrack.br(1,:),TestTrack.br(2,:),'k','lineWidth',1.5);
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'k--','lineWidth',1);

%%%%% plot trajectory of mass of center %%%%%
plot(traj.x,traj.y,'r-')

%%%%% plot car body with two tires (end of the moment) %%%%%
wheelscale = 2;
Car.Rw = 0.33;
Car.a=1.35; Car.b=1.45;
Xcar=[traj.x(end);traj.y(end);0];
tv=[cos(traj.psi(end));sin(traj.psi(end));0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];

pfront=Car.a*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];
prear=-Car.b*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];


Rotate=[cos(traj.psi(end)+input.steer(end)) -sin(traj.psi(end)+input.steer(end))  0
        sin(traj.psi(end)+input.steer(end))  cos(traj.psi(end)+input.steer(end))  0
        0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',6);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',2);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',4);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',4);
xlim([200,600])
ylim([-200,200])
% text(traj.x(end)+20,traj.y(end)+5,{str1})

%%%%% plot some marker points %%%%%
plot(traj.x(500),traj.y(500),'rx','LineWidth',2)
text(traj.x(500),traj.y(500),'      \leftarrow This is the 5 second position')
plot(traj.x(750),traj.y(750),'rx','LineWidth',2)
text(traj.x(750),traj.y(750),'      \leftarrow This is the 7.5 second position')


%%%%% plot some arc length to check %%%%%
plot(pinpointed_xr_(600),pinpointed_yr_(600),'bx','lineWidth',1.5);hold on;
plot(pinpointed_xl_(600),pinpointed_yl_(600),'bx','lineWidth',1.5);hold on;
plot(traj.x(600),traj.y(600),'ro','lineWidth',1.5); hold on;

plot(pinpointed_xr_(300),pinpointed_yr_(300),'bx','lineWidth',1.5);hold on;
plot(pinpointed_xl_(300),pinpointed_yl_(300),'bx','lineWidth',1.5);hold on;
plot(traj.x(300),traj.y(300),'ro','lineWidth',1.5);

plot(pinpointed_xr_(1200),pinpointed_yr_(1200),'bx','lineWidth',1.5);hold on;
plot(pinpointed_xl_(1200),pinpointed_yl_(1200),'bx','lineWidth',1.5);hold on;
plot(traj.x(1200),traj.y(1200),'ro','lineWidth',1.5);

%% Figure 2
figure(2)
%%%%% plot total track %%%%%
hold on;axis equal;box on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k','lineWidth',1.5);
plot(TestTrack.br(1,:),TestTrack.br(2,:),'k','lineWidth',1.5);
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'k--','lineWidth',1);

%%%%% plot trajectory of mass of center %%%%%
plot(traj.x,traj.y,'r-')

%%%%% plot car body with two tires (end of the moment) %%%%%
wheelscale = 2;
Car.Rw = 0.33;
Car.a=1.35; Car.b=1.45;
Xcar=[traj.x(end);traj.y(end);0];
tv=[cos(traj.psi(end));sin(traj.psi(end));0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];

pfront=Car.a*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];
prear=-Car.b*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];


Rotate=[cos(traj.psi(end)+input.steer(end)) -sin(traj.psi(end)+input.steer(end))  0
        sin(traj.psi(end)+input.steer(end))  cos(traj.psi(end)+input.steer(end))  0
        0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',6);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',2);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',4);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',4);
xlim([traj.x(end)-40,traj.x(end)+40])
ylim([traj.y(end)-40,traj.y(end)+40])
text(traj.x(end)-20,traj.y(end)+20,{str1})

%% Figure 1
figure(1) % zoom in version of figure (1)
%%%%% plot total track %%%%%
hold on;axis equal;box on;
plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k','lineWidth',1.5);
plot(TestTrack.br(1,:),TestTrack.br(2,:),'k','lineWidth',1.5);
plot(TestTrack.cline(1,:),TestTrack.cline(2,:),'k--','lineWidth',1);
%%%%% plot trajectory of mass of center %%%%%
plot(traj.x,traj.y,'r-')

%%%%% plot car body with two tires (end of the moment) %%%%%
wheelscale = 2;
Car.Rw = 0.33;
Car.a=1.35; Car.b=1.45;
Xcar=[traj.x(end);traj.y(end);0];
tv=[cos(traj.psi(end));sin(traj.psi(end));0];
vline=[Xcar-Car.b*tv,Xcar+Car.a*tv];

pfront=Car.a*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];
prear=-Car.b*[cos(traj.psi(end));sin(traj.psi(end));0]+[traj.x(end);traj.y(end);0];

Rotate=[cos(traj.psi(end)+input.steer(end)) -sin(traj.psi(end)+input.steer(end))  0
        sin(traj.psi(end)+input.steer(end))  cos(traj.psi(end)+input.steer(end))  0
        0             0           1];
frontwheelline=[pfront+Rotate*[-wheelscale*Car.Rw;0;0],pfront+Rotate*[wheelscale*Car.Rw;0;0]];
rearwheelline=[prear+wheelscale*Car.Rw*tv,prear-wheelscale*Car.Rw*tv];
L(1)=plot3(Xcar(1),Xcar(2),Xcar(3),'bo','MarkerSize',10);
L(2)=plot3(vline(1,:),vline(2,:),vline(3,:),'b','LineWidth',3);
L(3)=plot3(frontwheelline(1,:),frontwheelline(2,:),frontwheelline(3,:),'g','LineWidth',5);
L(4)=plot3(rearwheelline(1,:),rearwheelline(2,:),rearwheelline(3,:),'g','LineWidth',5);
% text(traj.x(end),traj.y(end),'\leftarrow sin(\pi)')
xlim([traj.x(end)-10,traj.x(end)+10])
ylim([traj.y(end)-10,traj.y(end)+10])
text(traj.x(end)-5,traj.y(end)+8,{str1})
