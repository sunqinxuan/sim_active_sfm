% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.23,2018
% Filename: trans_inv.m
% Description: 
% ===========================
function Tcg=trans_inv(Tgc)
Rgc=Tgc(1:3,1:3);
tgc=Tgc(1:3,4);
Tcg=[Rgc',-Rgc'*tgc;0,0,0,1];