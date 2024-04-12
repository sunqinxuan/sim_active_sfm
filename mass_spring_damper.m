% ===========================
% Author: Sun Qinxuan
% Last modified: May.23,2018
% Filename: mass_spring_damper.m
% Description: 
% ===========================
function dx=mass_spring_damper(t,x,zeta,w)
% x=[x,x_dot];
dx=[x(2);-2*zeta*w*x(2)-w^2*x(1)];