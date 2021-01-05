yalmip('clear')
clear all
close all
clc


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
% nu   = size(B,2);
% ny   = size(C,1);

%simulation steps
nsteps      = 50;

x_hist      = zeros(nx,nsteps);
% x_hat_hist  = zeros(nx,nsteps);

% d_hat_hist  = zeros(1,nsteps);
u_hist      = zeros(nu,nsteps-1);

x_hist(:,1)     = [0; 0; 0; 2]; %Initial condition %different from exercise_5_solution
% x_hat_hist(:,1) = [3;0]; %Initial estimate


for i = 1:nsteps-1
    fprintf('step %i \n',i);
    
    x = x_hist(:,i+1);
    
    ux = mpc_x.get_u(x); %given in the desciption project
    u_hist(:,i) = ux;
    
    %% Apply to the plant
    
    x_hist(:,i+1) = A*x_hist(:,i) + B*u_hist(:,i);
%     y             = C*x_hist(:,i) + d            ;
end


% % Get control inputs with
% ux = mpc_x.get_u(x);
