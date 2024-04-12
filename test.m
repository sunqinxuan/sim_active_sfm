clc;clear;
% close all;

x0=[1/4-1/3;0];
delta_t=10;
zeta=1;
w=sqrt(1.3); % sigma
options=odeset('reltol',1e-8);
[t,y]=ode45(@mass_spring_damper,[0,delta_t],x0,options,zeta,w);
figure;
plot(t,y(:,1));

% H = tf([1],[1 10 10]);
% step(H);
% grid;
% Kp = 100;
% Kd = 10;
% G = tf([Kd Kp],1);
% t = 0:0.01:10;
% L = feedback(G*H,1);
% figure; step(L,t);

%%
A=zeros(4,4);A(1,2)=1;A(2,1)=1;A(3,4)=1;A(4,3)=-1;
B=zeros(4,2);B(1,1)=1;B(4,2)=1;
C=zeros(2,4);C(1,1)=1;C(2,3)=1;
D=zeros(2,2);
sys = ss(A,B,C,D);
% P=[30 5; 30 5];
% Q=[30 -20; 30 -20];
num = {[-30,-30],[20,-5];[-30,-30],[20,-5]};
den = {1,1;1,1};
ctl=tf(num,den);
t = 0:0.1:20;
L = feedback(sys,ctl);
figure; step(ctl,t);

%%

%设一被控对象G（s）=50/(0.125s^2+7s),
%用增量式PID控制算法编写仿真程序
%（输入分别为单位阶跃、正弦信号，采样时间为1ms，控制器输出限幅：[-5,5],
%  仿真曲线包括系统输出及误差曲线，并加上注释、图例）。

clear all;
close all; 
ts=0.001;                 %采样时间
sys=tf(50,[0.125,7, 0]); %tf是传递函数  即被控对象函数G（）;
dsys=c2d(sys,ts,'z');    %把控制函数离散化
[num,den]=tfdata(dsys,'v');% 离散化后提取分子、分母  
u_1=0.0;
u_2=0.0;
y_1=0.0;
y_2=0.0;
x=[0,0,0]';
error_1=0;
error_2=0;
for k=1:1:1000
    time(k)=k*ts;                        %采样次数
    S=1;
    if S==1
        kp=10;ki=0.1;kd=15;             %初始化PID  
        rin(k)=1;                       %Step Signal 
    elseif S==2
        kp=10;ki=0.1;kd=15;             
        rin(k)=0.5*sin(2*pi*k*ts);       %Sine Signal     即实际输入    
    end 
    du(k)=kp*x(1)+kd*x(2)+ki*x(3);      %PID Controller   控制系数  
    u(k)=u_1+du(k);                     %Restricting the output of controller
    if u(k)>=5       
       u(k)=5;
    end
    if u(k)<=-5
       u(k)=-5;
    end
    %Linear model
    yout(k)=-den(2)*y_1-den(3)*y_2+num(2)*u_1+num(3)*u_2;          %实际输出
    error(k)=rin(k)-yout(k);                                       %Return of parameters 误差
    u_2=u_1;                                                       %保存上上次输入   为下次计算
    u_1=u(k);                                                      %保存上一次控制系数   为下次计算
    y_2=y_1;                                                       %保存上上次次输出   为下次计算
    y_1=yout(k);                                                   %保存上一次输出   为下次计算
    x(1)=error(k)-error_1;                                         %Calculating P
    x(2)=error(k)-2*error_1+error_2;                               %Calculating D
    x(3)=error(k);                                                 %Calculating I 
    error_2=error_1;                    
    error_1=error(k);                    
end
figure(1);
plot(time,rin,'b',time,yout,'r');                        %输入 和实际控制输出
xlabel('time(s)'),ylabel('rin,yout'); 
figure(2);
plot(time,error,'r')                                     %输入与输出误差输出曲线
xlabel('time(s)');ylabel('error');
