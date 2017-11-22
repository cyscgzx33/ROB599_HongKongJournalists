%% W2017 ME 542 Project: Formula 1 car racing: subfunctions
%   Copyright (C)2017 Chaozhe He. All Rights Reserved.
%   Author: Chaozhe He
%           Department of Mechanical Engineering
%           University of Michigan, Ann Arbor
%           March, 2017
% Version 1.0
% Gives the laterial forces model for rear wheel drive model
% Any issues/bug reports,
% please email to hchaozhe@umich.edu
function [Ffl,Frl]=Force_rwd(sigma,R,gamma,gammadot,m,m0,d,L)
Frl= -m/L*(1-d/L)*tan(gamma).*sigma.^2 ...
     +1/L./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)...
    .*(m*(m0*L-m*d)*gammadot.*sigma ...
      +(m0*L-m*d)*R.*sin(gamma).*cos(gamma)) ;
  
Ffl= -m*d/L^2*tan(gamma)./cos(gamma).*sigma.^2 ...
       -m*m0*gammadot.*sigma./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2)./(cos(gamma)) ...
       -m0*R.*sin(gamma)./(m*(cos(gamma)).^2+m0*(sin(gamma)).^2);