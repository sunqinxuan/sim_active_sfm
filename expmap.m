% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.23,2018
% Filename: expmap.m
% Description: 
% ===========================
function T=expmap(xi,theta)
v=xi(1:3);
w=xi(4:6);
if norm(w)==0
    T=[eye(3),v*theta;0,0,0,1];
else
    R=expmap_rot(w,theta);
    w=w/norm(w);
    t=(eye(3)-R)*skew_sym(w)*v+w*w'*v*theta;
    T=[R,t;0,0,0,1];
end