% ======================================================
% Author: Sun Qinxuan
% Last modified: May,8,2018
% Filename: transform_vel.m
% Description: 
%   control input (velocity represented in the camera frame)
%   transform into the velocity twist represented in the world frame.
% ======================================================
function xi=transform_vel(Tgc,u)
Rgc=Tgc(1:3,1:3);
tgc=Tgc(1:3,4);
trans=[Rgc,skew_sym(tgc)*Rgc;zeros(3),Rgc];
xi=trans*u;