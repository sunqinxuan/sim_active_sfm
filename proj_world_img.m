% ===========================
% Author: Sun Qinxuan
% Last modified: May.10,2018
% Filename: proj_world_img.m
% Description: 
% ===========================
function p=proj_world_img(Tgc,Pg,focal_length)
% focal_length (in pixels)
Tcg=trans_inv(Tgc);
Pc=transform_pt(Tcg,Pg);
p=[focal_length*Pc(1)/Pc(3);focal_length*Pc(2)/Pc(3)];