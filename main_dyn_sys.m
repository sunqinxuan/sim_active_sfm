close all;
clear;clc;
% focal length is 1;
Pg=[4;0;0]; % 3D point;
% velocity in the camera frame;
v=[-0.5;-0.5;0]; w=[0.5;0.6;-0.1];
% control input;
u=[v;w];
% camera pose;
Rgc=[ 0, 0, 1;
     -1, 0, 0;
      0,-1, 0]; % eye(3);
tgc=[0;0;0];
Tgc=[Rgc,tgc;0,0,0,1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options=odeset('reltol',1e-8);
last_time=1;
x0=[0;0;1/4];
% [t_pt,y_pt]=ode45(@dyn_sys,[0,last_time],x0,options,u);
% [t_pt,y_pt]=ode45(@point_dyn,[0,last_time],x0,options,u,Tgc);


%%%%%%%%%%%%%
f=figure;
set(f,'position',[100 0 1200 800]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% N=size(t_pt,1); % floor(last_time/delta_t);
time=0:0.01:1;
delta_t=0.01;
N=size(time,2);
M = moviein(N);
% Tgc_last=Tgc;
for i = 1:N
    if i==1
        xi=[v;w];
        Tcg=trans_inv(Tgc);
        Pc=transform_pt(Tcg,Pg);
        p=Pc/Pc(3); % [X/Z;Y/Z;1]
        %%%%%%%%%%%%%
        subplot(2,2,1);
        h_3d=gca;
        draw(h_3d,Tgc);
        scatter3(h_3d,Pg(1),Pg(2),Pg(3),'b');hold on;
        axis([-5,5,-5,5,-5,5]);grid on;
        xlabel('X');ylabel('Y');zlabel('Z');
        %%%%%%%%%%%%%%
        subplot(2,2,2);
        h_img=gca;
        scatter(h_img,p(1),-p(2),'b');
        axis('equal');axis([-1,1,-1,1]);
        xlabel('u');ylabel('v');grid on;
        %%%%%%%%%%%%%%
        % subplot(2,2,3);
        % h_depth=gca;
        % plot(h_depth,p(1),-p(2),'b');
        % axis('equal');axis([0,1,0,1]);
        % xlabel('t');ylabel('depth error');grid on;
    else
%         delta_t=t_pt(i)-t_pt(i-1);
        xi=transform_vel(Tgc_last,u);
        Tgc=expmap(xi,delta_t)*Tgc;
        Tcg=trans_inv(Tgc);
        Pc=transform_pt(Tcg,Pg);
        p=Pc/Pc(3); % [X/Z;Y/Z;1]
        fprintf('%d - %f\n',i,delta_t);
        fprintf('Pc - %f,%f,%f\n',Pc(1),Pc(2),Pc(3));
        fprintf('p - %f,%f\n',p(1),p(2));
        cla(h_3d);cla(h_img);
        draw(h_3d,Tgc);
        scatter3(h_3d,Pg(1),Pg(2),Pg(3),'b');hold on;
        scatter(h_img,p(1),-p(2),'b');hold on;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x0=[Pc_last(1)/Pc_last(3);Pc_last(2)/Pc_last(3);1/Pc_last(3)];
        [t_pt,y_pt]=ode45(@dyn_sys,[0,delta_t],x0,options,u);
        scatter(h_img,y_pt(size(y_pt,1),1),-y_pt(size(y_pt,1),2),'r');hold on;
        fprintf('Pc - %f,%f,%f\n\n',y_pt(size(y_pt,1),1),y_pt(size(y_pt,1),2),1/y_pt(size(y_pt,1),3));
    end
    Tgc_last=Tgc;
    Pc_last=transform_pt(Tcg,Pg);
	M(i) = getframe;
end