% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.18,2018
% Filename: logmap_rot.m
% Description: log map for SO(3).
% ===========================
function omega=logmap_rot(R)
theta=1/(cos((trace(R)-1.0)/2.0));
w=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
w=w/(2*sin(theta));
omega=w*theta;