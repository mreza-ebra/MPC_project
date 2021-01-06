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

%Design MPC Controller
mpc_x = MPC_Control_x(sys_x,Ts);
mpc_y = MPC_Control_y(sys_y,Ts);
mpc_z = MPC_Control_z(sys_z,Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw,Ts);
ctrl = quad.merge_controllers(mpc_x, mpc_y, mpc_z, mpc_yaw);

% Initial conditions
sol.u(:,1) = zeros(4,1); %Actual inputs
sol.x(:,1) = zeros(12,1);

ref = 1;
xref = [ref; ref; ref; deg2rad(20)];

% simulate for 10 seconds
for i = 1:length(t) - 1
    sol.u(:,i) = ctrl(sol.x(:,i), xref);
    % Get transformed control inputs v
    sol.vx(:,i) = mpc_x.get_u(sol.x([2, 5, 7, 10],i),ref); 
    sol.vy(:,i) = mpc_y.get_u(sol.x([1, 4, 8, 11],i),ref);
    sol.vz(:,i) = mpc_z.get_u(sol.x([9, 12],i),ref);
    sol.vyaw(:,i) = mpc_yaw.get_u(sol.x([3, 6],i),xref(4));
    
    % Compute next states
    sol.x(:,i+1) = quad.step(sol.x(:,i), sol.u(:,i), Ts);
end

sim = struct();
sim.x = t(1:end-1)';
sim.y = sol.x;

quad.plot(sim,sol.u')


