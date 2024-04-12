% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.17,2018
% Filename: controller_linear_vel.m
% Description: 
% ===========================
function dv=controller_linear_vel(t,v,pixel,kappa_des,k1,k2)
vx=v(1);vy=v(2);vz=v(3);
x=pixel(1);y=pixel(2);
Jv1=[vx-x*vz,vy-y*vz,(x*vz-vx)*x+(y*vz-vy)*y];
if norm(v)==0
    v2=1;
else
    v2=v'*v;
end
kappa=0.5*v2;
dv=k1*(kappa_des-kappa)*v/v2+k2*(eye(3)-v*v'/v2)*Jv1';