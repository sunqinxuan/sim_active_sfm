% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.17,2018
% Filename: point_dyn.m
% Description: System model.
% ===========================
function dx=point_dyn(t,x,u,Tgc0)
v=u(1:3);w=u(4:6);
dx=-v-cross(w,x);
% Tgc=expmap(u,t)*Tgc0;
% Rgc=Tgc(1:3,1:3);
% tgc=Tgc(1:3,4);
% % dx=-Rgc'*(cross(w,Rgc*x+tgc)+v);
% dx=-Rgc'*(cross(w,[4;0;0])+v);