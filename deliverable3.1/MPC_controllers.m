yalmip('clear')
clear all
close all
clc

umax = 1.5;
umin = 0;

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
% Design MPC controller

% mpc_x = MPC_Control_x(sys_x, Ts);
mpc_x = MPC_Control_y(sys_y, Ts);
% mpc_x = MPC_Control_z(sys_z, Ts);
% mpc_x = MPC_Control_yaw(sys_yaw, Ts);

A = mpc_x.A; %don't forget the "_x" indice
B = mpc_x.B;
C = mpc_x.C;
D = mpc_x.D;

nx   = size(A,1);
nu   = size(B,2);
% ny   = size(C,1);

%simulation steps
nsteps      = 50;

x_hist      = zeros(nx,nsteps);
% x_hat_hist  = zeros(nx,nsteps);

% d_hat_hist  = zeros(1,nsteps);

% u_hist      = zeros(nu,nsteps-1);
u_hist      = zeros(nu,nsteps);

x_hist(:,1)     = [0; 0; 0; 2]; %Initial condition %different from exercise_5_solution
% x_hat_hist(:,1) = [3;0]; %Initial estimate


for i = 1:nsteps-1
    fprintf('step %i \n',i);
    
    x = x_hist(:,i);
    
    ux = mpc_x.get_u(x); %given in the desciption project
    u_hist(:,i) = ux;
    
    %% Apply to the plant
    
    x_hist(:,i+1) = A*x_hist(:,i) + B*u_hist(:,i);
%     y             = C*x_hist(:,i) + d            ;
end

% % Get control inputs with
% ux = mpc_x.get_u(x);

%% Plotting the results

%comment or uncomment the part depending if you need them or not

figure
plot(u_hist); hold on
plot(umax*ones(size(u_hist)),'--');
plot(umin*ones(size(u_hist)),'--'); 
legend('u','u_max','u_min');

figure
hold on; grid on;

time = 0:Ts:((nsteps-1)*Ts);

settling_time = 8;
t= 0:Ts:((nsteps-1)*Ts);

% subplot(5,1,1)
% hold on; grid on;
% % title('System x')
% title('System y')
% plot(time, x_hist(1,:),'-k','markersize',20,'linewidth',2);
% % ylabel('$\dot{\beta}[rad/s]$','interpreter','latex');
% ylabel('$\dot{\alpha}[rad/s]$','interpreter','latex');
% 
% subplot(5,1,2)
% hold on; grid on;
% plot(time, x_hist(2,:),'-k','markersize',20,'linewidth',2);
% % ylabel('$\beta[rad]$','interpreter','latex');
% ylabel('$\alpha[rad]$','interpreter','latex');
% 
% subplot(5,1,3)
% hold on; grid on;
% plot(time, x_hist(3,:),'-k','markersize',20,'linewidth',2);
% % ylabel('$\dot{x}[m/s]$','interpreter','latex');
% ylabel('$\dot{y}[m/s]$','interpreter','latex');
% 
% subplot(5,1,4)
% hold on; grid on;
% plot(time, x_hist(4,:),'-k','markersize',20,'linewidth',2);
% % ylabel('$x[m]$','interpreter','latex');
% ylabel('$y[m]$','interpreter','latex');
% 
% subplot(5,1,5)
% hold on; grid on;
% plot(time, u_hist,'-k','markersize',20,'linewidth',2);
% ylabel('$input[N.m]$','interpreter','latex');
% xlabel('$time[s]$','interpreter','latex');


subplot(3,1,1)
hold on; grid on;
% title('System yaw')
title('System z')
plot(time, x_hist(3,:),'-k','markersize',20,'linewidth',2);
ylabel('$\dot{z}[m/s]$','interpreter','latex');
% ylabel('$\dot{yaw}[m/s]$','interpreter','latex');

subplot(3,1,2)
hold on; grid on;
plot(time, x_hist(4,:),'-k','markersize',20,'linewidth',2);
ylabel('$z[m]$','interpreter','latex');
% ylabel('$yaw[m]$','interpreter','latex');

subplot(3,1,3)
hold on; grid on;
plot(time, u_hist,'-k','markersize',20,'linewidth',2);
ylabel('$input[N.m]$','interpreter','latex');
xlabel('$time[s]$','interpreter','latex');