yalmip('clear')
clear
close all
clc


Ts = 0.2;
Tfinal = 10;
t = 0:Ts:Tfinal;
quad = Quad();
[xs,us] = quad.trim();        % Compute steady−state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

mpc_x = MPC_Control_x(sys_x, Ts);
mpc_y = MPC_Control_y(sys_y, Ts);
mpc_z = MPC_Control_z(sys_z, Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts);

% Simulate the quadcopter and plot trajectory
sim = quad.sim(mpc_x, mpc_y, mpc_z, mpc_yaw);
quad.plot(sim);