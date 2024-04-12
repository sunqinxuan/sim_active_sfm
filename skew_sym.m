% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.18,2018
% Filename: skew_sym.m
% Description: skew symmetric matrix w.r.t. a vector.
% ===========================
function W=skew_sym(w)
W=zeros(3,3);
W(1,2)=-w(3);
W(2,1)=w(3);
W(1,3)=w(2);
W(3,1)=-w(2);
W(2,3)=-w(1);
W(3,2)=w(1);
