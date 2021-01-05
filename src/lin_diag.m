clear all;
clc;
Ts = 0.2;
quad = Quad();
[xs,us] = quad.trim();        % Compute steady−state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model
sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%Design MPC Controller
mpc_x = MPC_Control_x(sys_x,Ts);
mpc_y = MPC_Control_y(sys_y,Ts);

x = zeros(4,1);
x(4) = 2;
% Get control inputs with
ux = mpc_x.get_u(x);
uy = mpc_y.get_u(x);