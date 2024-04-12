% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.18,2018
% Filename: expmap_rot.m
% Description: Exponential map for SO(3).
% ===========================
function R=expmap_rot(omega,theta)
% theta=norm(omega);
if norm(omega)~=0
    w=omega/norm(omega);
else
    w=omega;
end
theta=theta*norm(omega);
R=eye(3)+skew_sym(w)*sin(theta)+skew_sym(w)*skew_sym(w)*(1-cos(theta));