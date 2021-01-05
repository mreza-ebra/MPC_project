clear all;
clc;
Ts = 0.2;
Tfinal = 10;
t = 1:Ts:Tfinal;
quad = Quad();
[xs,us] = quad.trim();        % Compute steady−state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model
sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

%Design MPC Controller
mpc_x = MPC_Control_x(sys_x,Ts);
mpc_y = MPC_Control_y(sys_y,Ts);
mpc_z = MPC_Control_z(sys_z,Ts);
mpc_yaw = MPC_Control_yaw(sys_yaw,Ts);

% Initial conditions
sol.u(:,1) = zeros(4,1); %Actual inputs for the ode
sol.v(:,1) = zeros(4,1);
sol.x(:,1) = zeros(12,1);
sol.x([10,11,12],1) = 2;
sol.x(6,1) = deg2rad(45);

% simulate for 10 seconds
for i = 1:length(t)
    
    % Get transformed control inputs v
    sol.vx(:,i) = mpc_x.get_u(sol.x([2, 5, 7, 10],i)); 
    sol.vy(:,i) = mpc_y.get_u(sol.x([1, 4, 8, 11],i));
    sol.vz(:,i) = mpc_z.get_u(sol.x([9, 12],i));
    sol.vyaw(:,i) = mpc_yaw.get_u(sol.x([3, 6],i));
    
    % Group trasnformed control inputs v
    sol.v(:,i) = [sol.vz(:,i); sol.vy(:,i); sol.vx(:,i); sol.vyaw(:,i)];
    
    % Apply inverse transformation to get actual control input u (for
    % simulation)
    sol.u(:,i) = quad.T\sol.v(:,i) + us;
    
    % Compute next states
    sol.x([2, 5, 7, 10],i+1) = sys_xd.A*sol.x([2, 5, 7, 10],i) + sys_xd.B*sol.vx(:,i);
    sol.x([1, 4, 8, 11],i+1) = sys_yd.A*sol.x([1, 4, 8, 11],i) + sys_yd.B*sol.vy(:,i);
    sol.x([9, 12],i+1) = sys_zd.A*sol.x([9, 12],i) + sys_zd.B*sol.vz(:,i);
    sol.x([3, 6],i+1) = sys_yawd.A*sol.x([3, 6],i) + sys_yawd.B*sol.vyaw(:,i);
end


