%% W2017 ME 542 Project: Formula 1 car racing: subfunctions
%   Copyright (C)2017 Chaozhe He. All Rights Reserved.
%   Author: Chaozhe He
%           Department of Mechanical Engineering
%           University of Michigan, Ann Arbor
%           March, 2017
% Version 1.0
% Localization function
% Convert Cartisan coordinate x y psi to
% the descrption relative to track ribbon.
% convert the x,y,psi global description to s n local description
% Track.cline is used as the reference
% Any issues/bug reports,
% please email to hchaozhe@umich.edu
function [s,sf,sr,n,nf,nr]=Cartisan2Track(x,y,psi,s,Track,Car)
ntol=1e-8;
%%%%%%%%%%%%% This part may requires some detunning %%%%%%%%%%%%%
sseg=s+(-10:0.01:20);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%  center of mass  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% center
Xc=Track.center(sseg);
Normc=sqrt((Xc(1,:)-x).^2+(Xc(2,:)-y).^2);
[n,indexc]=min(Normc);  % the minimum value gives n
s=sseg(indexc);
tcenter=Track.t(s);
tcenter=tcenter(1:2);
% determine whether n is postive or native
vec_c=[x;y]-Xc(1:2,indexc);
valuec=det([vec_c';tcenter']); % from the path2center vector to the tangent vector at the center
if valuec>ntol  % path2center vector pointing towards the right need to turn a postive angle to over come
    n=-n;
else if valuec>=-ntol  % to small, the center is almost on the path, no need to adjust. 
    n=0; 
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  front axis  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pfront=Car.a*[cos(psi);sin(psi);0]+[x;y;0];
Xf=Xc(:,indexc:end);
Normf=sqrt((Xf(1,:)-pfront(1)).^2+(Xf(2,:)-pfront(2)).^2);
[nf,indexf]=min(Normf);  
sf=sseg(indexf-1+indexc);
tfront=Track.t(s);
tfront=tfront(1:2);
% determine whether n is postive or native
vec_f=pfront(1:2)-Xf(1:2,indexf);
valuef=det([vec_f';tfront']); % from the path2center vector to the tangent vector at the center
if valuef>ntol  % path2center vector pointing towards the right need to turn a postive angle to over come
    nf=-nf;
else if valuef>=-ntol  % to small, the center is almost on the path, no need to adjust. 
       nf=0; 
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  rear axis  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
prear=-Car.b*[cos(psi);sin(psi);0]+[x;y;0];
Xr=Xc(:,1:indexc);
Normr=sqrt((Xr(1,:)-prear(1)).^2+(Xr(2,:)-prear(2)).^2);
[nr,indexr]=min(Normr);  % the minimum value gives nr
sr=sseg(indexr);
trear=Track.t(s);
trear=trear(1:2);
% determine whether n is postive or native
vec_r=prear(1:2)-Xr(1:2,indexr);
valuer=det([vec_r';trear']); % from the path2center vector to the tangent vector at the center
if valuer>ntol  % path2center vector pointing towards the right need to turn a postive angle to over come
    nr=-nr;
else if valuer>=-ntol  % to small, the center is almost on the path, no need to adjust. 
       nr=0; 
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end