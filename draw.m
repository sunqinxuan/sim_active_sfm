% ===========================
% Author: Sun Qinxuan
% Last modified: Apr.23,2018
% Filename: draw.m
% Description: 
% ===========================
function draw(h,Tgc)
Rgc=Tgc(1:3,1:3);
tgc=Tgc(1:3,4);
O=tgc; % origin of the camera frame;
Ac=[1;1;1];   Bc=[1;-1;1]; 
Cc=[-1;-1;1]; Dc=[-1;1;1];
Ag=Rgc*Ac+tgc; Bg=Rgc*Bc+tgc;
Cg=Rgc*Cc+tgc; Dg=Rgc*Dc+tgc;
plot3(h,[O(1);Ag(1)],[O(2);Ag(2)],[O(3);Ag(3)],'r');hold on;
plot3(h,[O(1);Bg(1)],[O(2);Bg(2)],[O(3);Bg(3)],'r');hold on;
plot3(h,[O(1);Cg(1)],[O(2);Cg(2)],[O(3);Cg(3)],'r');hold on;
plot3(h,[O(1);Dg(1)],[O(2);Dg(2)],[O(3);Dg(3)],'r');hold on;
plot3(h,[Ag(1);Bg(1)],[Ag(2);Bg(2)],[Ag(3);Bg(3)],'r');hold on;
plot3(h,[Bg(1);Cg(1)],[Bg(2);Cg(2)],[Bg(3);Cg(3)],'r');hold on;
plot3(h,[Cg(1);Dg(1)],[Cg(2);Dg(2)],[Cg(3);Dg(3)],'r');hold on;
plot3(h,[Dg(1);Ag(1)],[Dg(2);Ag(2)],[Dg(3);Ag(3)],'r');hold on;
% draw axis
Xc=[1;0;0];Yc=[0;1;0];Zc=[0;0;1];
Xg=Rgc*Xc+tgc;Yg=Rgc*Yc+tgc;Zg=Rgc*Zc+tgc;
plot3(h,[O(1);Xg(1)],[O(2);Xg(2)],[O(3);Xg(3)],'g');hold on;
plot3(h,[O(1);Yg(1)],[O(2);Yg(2)],[O(3);Yg(3)],'b');hold on;
plot3(h,[O(1);Zg(1)],[O(2);Zg(2)],[O(3);Zg(3)],'r');hold on;