% ===========================
% Author: Sun Qinxuan
% Last modified: May.10,2018
% Filename: proj_world_cam.m
% Description: 
% ===========================
function Pc=proj_world_cam(Tgc,Pg)
Tcg=trans_inv(Tgc);
Pc=transform_pt(Tcg,Pg);