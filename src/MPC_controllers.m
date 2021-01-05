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
mpc_x = MPC_Control_x(sys_x, Ts);

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
figure
plot(u_hist); hold on
plot(umax*ones(size(u_hist)),'--');
plot(umin*ones(size(u_hist)),'--'); 
legend('u','u_max','u_min');

figure
hold on; grid on;

o = ones(1, nsteps);

settling_time = 8;
t= 0:Ts:((nsteps-1)*Ts);

subplot(5,1,1)
hold on; grid on;
plot(x_hist(1,:),'-k','markersize',20,'linewidth',2);
% plot(t, x_hist(1,:),'-k','markersize',20,'linewidth',2);
% plot(1:nsteps, 0.035*o,'r','linewidth',2)
% plot(1:nsteps,-0.035*o,'r','linewidth',2)
ylabel('Beta_dot')

subplot(5,1,2)
hold on; grid on;
plot(x_hist(2,:),'-k','markersize',20,'linewidth',2);
% plot(t, x_hist(2,:),'-k','markersize',20,'linewidth',2);
ylabel('Beta')

subplot(5,1,3)
hold on; grid on;
plot(x_hist(3,:),'-k','markersize',20,'linewidth',2);
% plot(t, x_hist(3,:),'-k','markersize',20,'linewidth',2);
ylabel('Velocity')

subplot(5,1,4)
hold on; grid on;
plot(x_hist(4,:),'-k','markersize',20,'linewidth',2);
% plot(t, x_hist(4,:),'-k','markersize',20,'linewidth',2);
ylabel('Position')

subplot(5,1,5)
hold on; grid on;
plot(u_hist,'-k','markersize',20,'linewidth',2);
% plot(t, u_hist,'-k','markersize',20,'linewidth',2);
ylabel('u_hist')

figure
      hold on; grid on;
      title('Terminal invariant set for x')
      
      subplot(3,1,1)
      hold on; grid on;
      Xf.projection(1:2).plot();
      xlabel('Beta_dot');
      ylabel('Beta');
      
      subplot(3,1,2)
      hold on; grid on;
      Xf.projection(2:3).plot();
      xlabel('Beta');
      ylabel('x_dot');
      
      subplot(3,1,3)
      hold on; grid on;
      Xf.projection(3:4).plot();
      xlabel('x_dot');
      ylabel('x');