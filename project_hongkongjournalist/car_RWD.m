%% W2017 ME 542 Project: Formula 1 car racing: subfunctions
%   Copyright (C)2017 Chaozhe He. All Rights Reserved.
%   Author: Chaozhe He
%           Department of Mechanical Engineering
%           University of Michigan, Ann Arbor
%           March, 2017
% Version 1.0
% Rear wheel drive 2D bicycle model.

% The states are 
%      x coordinate
%      y coordinate
%      psi yaw angle
%      sigma longitudinal speed
% The inputs are 
%      R driving force at rear wheel  
%      gamma steering angle gamma and 
%      gammadot time derivative of steering angle
% Car is the structure variable for Car parameters
% The outputs are two folds
%     dX gives the right handside of rearwheel drive model
%      U is the implicit values include control and the update of para  
%         The default order is 
%         U1=R,U2=gamma,U3=gammadot,U4=s,U5=sf,U6=sr,U7=n,U8=nf,U9=nr; 
%        you may change fun definition but ONLY ADD Y and U AFTER these default ones.
%  s0_prior is used in the sub function for faster calculation
% function Cartisan2Track is used to convert Cartisan coordinate x y psi to
% the descrption relative to track ribbon.
% Any issues/bug reports,
% please email to hchaozhe@umich.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dX,u]=car_RWD(t,x,y,psi,sigma,gamma,gammadot,R,Track,Car,s0_prior)
[s0,sf,sr,n0,nf,nr]=Cartisan2Track(x,y,psi,s0_prior,Track,Car);
tgamma=tan(gamma);
dx=(cos(psi)-Car.b/Car.w*sin(psi)*tgamma)*sigma;
dy=(sin(psi)+Car.b/Car.w*cos(psi)*tgamma)*sigma;
dpsi=sigma*tgamma/Car.w;
dtgamma=1/(cos(gamma))^2*gammadot;
dv=(R-Car.k*sigma^2-Car.m0*tgamma*dtgamma*sigma)/(Car.m+Car.m0*tgamma^2);
dX=[dx;dy;dpsi;dv];
u=[R;tgamma;dtgamma;s0;sf;sr;n0;nf;nr];
end