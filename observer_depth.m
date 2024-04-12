% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.17,2018
% Filename: observer_depth.m
% Description: depth observer.
% ===========================
function dx=observer_depth(t,x,u,xm,alpha,beta,D,xu)

% x=[s_hat;chi_hat];
% u=[v;w];
xm_hat=x(1:2); % pixel estimate;
xu_hat=x(3); % inverse depth estimate;
v=u(1:3);w=u(4:6); % control input;
xi=xm-xm_hat; % pixel error;
z=xu-xu_hat; % inverse depth error;

% gain matrices;
P=alpha*eye(2);
Lambda=beta*eye(1);
% H=eye(2);

x=xm(1);y=xm(2);
fm=[x*y,-(1+x^2),y;1+y^2,-x*y,-x]*w;
Omega=[x*v(3)-v(1),y*v(3)-v(2)];
fu=v(3)*xu_hat^2+(y*w(1)-x*w(2))*xu_hat;
fu_true=v(3)*xu^2+(y*w(1)-x*w(2))*xu;

[U,S,V]=svd(Omega); % S(1,1)=\sigma_1;
D(1)=2*sqrt(alpha*beta)*S(1,1); % critically damped;
D(2)=D(1);
H=V*diag(D)*V';

xm_hat_dot=fm+Omega'*xu_hat+H*xi;
xu_hat_dot=fu+Lambda*Omega*P*xi;
xi_dot=-H*xi+Omega'*z;
z_dot=-Lambda*Omega*P*xi+fu_true-fu;

dx=[xm_hat_dot;xu_hat_dot];