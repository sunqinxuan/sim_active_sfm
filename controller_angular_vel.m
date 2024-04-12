% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.17,2018
% Filename: controller_angular_vel.m
% Description: 
% ===========================
function dw=controller_angular_vel(t,w,pixel,k1)
theta=atan(norm(pixel));
w_unit=cross([pixel;1],[0;0;1]);
if norm(w_unit)==0
    dw=-k1*theta*w_unit;
else
    dw=-k1*theta*w_unit/norm(w_unit);
end;