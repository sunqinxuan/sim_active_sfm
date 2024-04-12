% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.23,2018
% Filename: transform_pt.m
% Description: 
% ===========================
function Pg=transform_pt(Tgc,Pc)
Rgc=Tgc(1:3,1:3);
tgc=Tgc(1:3,4);
Pg=Rgc*Pc+tgc;