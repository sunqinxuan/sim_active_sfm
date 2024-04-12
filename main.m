close all;
clear;clc;
f=1; % focal length is 1;
Pg=[4;0;0]; % 3D point;
% velocity in the camera frame;
v=[0.3;0.4;0.0]; w=[0.05;0.0;0.1];
% control input;
u0=[v;w];
% camera pose;
Rgc=[ 0, 0, 1;
     -1, 0, 0;
      0,-1, 0]; % eye(3);
tgc=[0;0;0];
Tgc=[Rgc,tgc;0,0,0,1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
options=odeset('reltol',1e-8);
% last_time=1;
% Pg0=[0;0;1/2];
% [t_pt,y_pt]=ode45(@dyn_sys,[0,last_time],x0,options,u);
% [t_pt,y_pt]=ode45(@point_dyn,[0,last_time],x0,options,u,Tgc);


%%%%%%%%%%%%%
f=figure;
set(f,'position',[100 0 1200 800]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% N=size(t_pt,1); % floor(last_time/delta_t);
delta_t=0.1;
time_last=10;
time=0:delta_t:time_last;
N=size(time,2);
M = moviein(N);
Pc_seq=zeros(N,3);
p_seq=zeros(N,2);
Tgc_seq=zeros(N,16);
pixel_obsv_seq=zeros(N,3);
u_seq=zeros(N,6);
xi_seq=zeros(N,6);
depth_error_seq=zeros(N,1);
g_t=zeros(N,1);
sigma_seq=zeros(N,1);
% error_seq=zeros(N,3);
% coefficients for observer;
alpha=1;
beta=1;
d=[2,2];
% coefficients
% linear velocity control;
kappa_des=0.02;
k1=10;
k2=10;
% coefficients
% angular velocity control;
k3=0.01;
P=10*ones(3,2);
Q=10*ones(3,2);
for i = 1:N
    if i==1
        xi=[v;w];
        Pc=proj_world_cam(Tgc,Pg);
        p=proj_world_img(Tgc,Pg,1);
        pixel_obsv=[p',1/2];
        depth_error=1/Pc(3)-pixel_obsv(3);
        u=u0;
        sigma=0;
%         pixel_obsv=[Pc(1)/Pc(3);Pc(2)/Pc(3);1/Pc(3)];
        %%%%%%%%%%%%%
        subplot(2,3,1);
        h_3d=gca;
        draw(h_3d,Tgc);
        scatter3(h_3d,Pg(1),Pg(2),Pg(3),'b');hold on;
        axis([-5,5,-5,5,-5,5]);grid on;
        xlabel('X');ylabel('Y');zlabel('Z');
        %%%%%%%%%%%%%%
        subplot(2,3,2);
        h_img=gca;
        scatter(h_img,p(1),-p(2),'b');hold on;
        axis('equal');axis([-1,1,-1,1]);
        xlabel('u');ylabel('v');grid on;
        %%%%%%%%%%%%%%
        subplot(2,3,4);
        h_depth_err=gca;hold on;
        % plot(h_depth,p(1),-p(2),'b');
%         axis('equal');
        axis([0,time_last,-1,1]);
        xlabel('t');ylabel('depth error');grid on;
        %%%%%%%%%%%%%%
        subplot(2,3,5);
        h_sigma=gca;hold on;
        % plot(h_depth,p(1),-p(2),'b');
%         axis('equal');
        axis([0,time_last,0,5]);
        xlabel('t');ylabel('\sigma^2');grid on;
        %%%%%%%%%%%%%%
        subplot(2,3,6);
        h_gt=gca;hold on;
        % plot(h_depth,p(1),-p(2),'b');
%         axis('equal');
        axis([0,time_last,-0.2,0.2]);
        xlabel('t');ylabel('g(e,t)');grid on;
    else
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         v0=u_last(1:3);
%         [t_v,y_v]=ode45(@controller_linear_vel,[0,delta_t],v0,options,p(1:2),kappa_des,k1,k2);
%         u(1:3)=y_v(size(y_v,1),:);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        J_w=Jacobian_w(p);
        if det(J_w'*J_w)~=0
            J_w_inv=(J_w'*J_w)\J_w';
            u(4:6)=-k3*J_w_inv*p;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         if i==2
%             u(4:6)=0;
%         else
%             u(4:6)=-P*p-Q*(p-p_seq(i-2,:)');
%         end
%         w0=u_last(4:6);
%         u(4:6)=controller_angular_vel(0,w0,p(1:2),k3);
%         [t_w,y_w]=ode45(@controller_angular_vel,[0,delta_t],w0,options,p(1:2),k3);
%         u(4:6)=y_w(size(y_w,1),:);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xi=transform_vel(Tgc_last,u);
        Tgc=expmap(xi,delta_t)*Tgc;
        Pc=proj_world_cam(Tgc,Pg);
        p=proj_world_img(Tgc,Pg,1);
        fprintf('%d - %f\n',i,delta_t);
        fprintf('Pc - %f,%f,%f\n',Pc(1),Pc(2),Pc(3));
        fprintf('p - %f,%f\n',p(1),p(2));
        cla(h_3d);
        cla(h_img);
        draw(h_3d,Tgc);
        scatter3(h_3d,Pg(1),Pg(2),Pg(3),'b');hold on;
        scatter(h_img,p(1),-p(2),'b');hold on;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        x0=pixel_last;
        xu=Pc(3);
        [t_pt,y_pt]=ode45(@observer_depth,[0,delta_t],x0,options,u,p,alpha,beta,d,xu);
        pixel_obsv=y_pt(size(y_pt,1),:);
%         error_seq(i,:)=y_pt(size(y_pt,1),4:6);
        scatter(h_img,y_pt(size(y_pt,1),1),-y_pt(size(y_pt,1),2),'r');hold on;
        fprintf('Pc - %f,%f,%f\n\n',y_pt(size(y_pt,1),1),y_pt(size(y_pt,1),2),1/y_pt(size(y_pt,1),3));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        depth_error=1/Pc(3)-pixel_obsv(3);
        scatter(h_depth_err,time(i),depth_error,'b.');hold on;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         g_t(i,1)=u(3)*(Pc(3)-1/pixel_obsv(3))*(Pc(3)+1/pixel_obsv(3))+(pixel_obsv(2)*u(4)-pixel_obsv(1)*u(5))*(Pc(3)-1/pixel_obsv(3));
        sigma=(p(1)*u(3)-u(1))^2+(p(2)*u(3)-u(2))^2;
        scatter(h_sigma,time(i),sigma,'b.');hold on;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        g_t(i,1)=u(3)*(1/Pc(3)-pixel_obsv(3))*(1/Pc(3)+pixel_obsv(3))+(pixel_obsv(2)*u(4)-pixel_obsv(1)*u(5))*(1/Pc(3)-pixel_obsv(3));
        scatter(h_gt,time(i),g_t(i,1),'b.');hold on;
    end
    Tgc_last=Tgc;
    Pc_last=Pc;
    pixel_last=pixel_obsv;
    u_last=u;
    Pc_seq(i,:)=Pc';
    p_seq(i,:)=p';
    Tgc_seq(i,:)=reshape(Tgc,1,16);
    pixel_obsv_seq(i,:)=pixel_obsv;
    u_seq(i,:)=u';
    xi_seq(i,:)=xi';
    depth_error_seq(i)=depth_error;
    sigma_seq(i,1)=sigma;
	M(i) = getframe;
end
draw_traj(h_3d,Tgc_seq);
x0=[1/4-1/2;0];
[t,y]=ode45(@mass_spring_damper,[0,time_last],x0,options,1,sqrt(sigma));
plot(h_depth_err,t,y(:,1),'r');