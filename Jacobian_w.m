% ===========================
% Author: Sun Qinxuan
% Last modified: May.23,2018
% Filename: Jacobian_w.m
% Description: 
% ===========================
function J=Jacobian_w(p)
x=p(1);y=p(2);
J=[x*y,-(1+x^2),y;1+y^2,-x*y,-x];