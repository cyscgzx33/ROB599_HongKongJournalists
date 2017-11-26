function [dot_prod, OutTrackFlag, s_act, s_ref] = checkIfOutTrack(traj)
%%%%%%%% This function is to check if the traj is out of the track %%%%%%
% Inputs:
% traj is an (n x 2) trajectory vector
%
% Outputs:
% dot_prod is the result of dot product of the designated vectors
% OutTrackFlag tells whether the traj has some parts out of track
%       OutTrackFlag = 1 means there are some parts out of the track
%       OutTrackFlag = 0 means all the trajectory parts are in the track
% s_act is the actual arc length
% s_ref is the center line arc length
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traj_x = traj(:,1);
traj_y = traj(:,2);
load('TestTrack');
%% Reference Arc Length
% calculate arc_ref, which is the reference arc length
arc_ref = zeros(size(TestTrack.cline,2),1);
for i = 1:size(TestTrack.cline,2)-1
    arc_ref(i+1) = sqrt((TestTrack.cline(1,i+1)-TestTrack.cline(1,i))^2+(TestTrack.cline(2,i+1)-TestTrack.cline(2,i))^2) + arc_ref(i);
end
s_ref = arc_ref;

%% Actual Arc Length
% derive some interp1 mapping
s = zeros(length(traj_x),1);
s(1) = 2.5; % initial offsite
for i = 1:length(traj_x)-1
    s(i+1) = sqrt((traj_x(i+1)-traj_x(i))^2+(traj_y(i+1)-traj_y(i))^2) + s(i);
end

%% Fine Tuning About the Actual Arc Length Corresponding to the Reference
s_ = s;
for i = 1:length(traj_x)-1
    ss = s_(i) + (-10:0.01:20)';
    Xc(1,:) = interp1(arc_ref,TestTrack.cline(1,:),ss);
    Xc(2,:) = interp1(arc_ref,TestTrack.cline(2,:),ss);
    Normc=sqrt((Xc(1,:)-traj_x(i)).^2+(Xc(2,:)-traj_y(i)).^2);
    [~,indexc]=min(Normc);  % the minimum value gives n
    s_(i)=ss(indexc);
end
s_act = s_;

%% Finding Corresponding Points on the Left/Right Boundaries using Interpolation
corr_xl = interp1(arc_ref,TestTrack.bl(1,:),s);
corr_yl = interp1(arc_ref,TestTrack.bl(2,:),s);
corr_xr = interp1(arc_ref,TestTrack.br(1,:),s);
corr_yr = interp1(arc_ref,TestTrack.br(2,:),s);

%% Check whether the traj is out of the track 
% derive the vectors pointing to pinpointed points of left/right boundaries
vector_bl = [corr_xl,corr_yl] - [traj_x,traj_y];
vector_br = [corr_xr,corr_yr] - [traj_x,traj_y];

dot_prod = dot(vector_bl',vector_br');

if dot_prod <= 0
    OutTrackFlag = 0;
else
    OutTrackFlag = 1;
end
end