% ===========================
% Author: Sun Qinxuan
% Last modified: May.10,2018
% Filename: draw_traj.m
% Description: 
% ===========================
function draw_traj(h,Tgc_seq)
N=size(Tgc_seq,1);
for i=1:N
    Tgc=reshape(Tgc_seq(i,:),4,4);
    Rgc=Tgc(1:3,1:3);
    tgc=Tgc(1:3,4);
    O=tgc; % origin of the camera frame;
    Xc=[1;0;0];Yc=[0;1;0];Zc=[0;0;1];
    Xg=Rgc*Xc+tgc;Yg=Rgc*Yc+tgc;Zg=Rgc*Zc+tgc;
    % plot3(h,[O(1);Xg(1)],[O(2);Xg(2)],[O(3);Xg(3)],'g');hold on;
    % plot3(h,[O(1);Yg(1)],[O(2);Yg(2)],[O(3);Yg(3)],'b');hold on;
    plot3(h,[O(1);Zg(1)],[O(2);Zg(2)],[O(3);Zg(3)],'r');hold on;
end