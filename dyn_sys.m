% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.17,2018
% Filename: dyn_sys.m
% Description: System model.
% ===========================
function dx=dyn_sys(t,x,u)
% x=[s;chi];
% u=[v;w];
s=x(1:2);chi=x(3);
v=u(1:3);w=u(4:6);
x=s(1);y=s(2);
fm=[x*y,-(1+x^2),y;1+y^2,-x*y,-x]*w;
Omega=[x*v(3)-v(1),y*v(3)-v(2)];
fu=v(3)*chi^2+(y*w(1)-x*w(2))*chi;
dx=[fm+Omega'*chi;fu];