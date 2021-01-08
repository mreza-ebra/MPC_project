function [ctrl, traj] = ctrl_NMPC(quad)

import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 20; % MPC horizon [SET THIS VARIABLE]
% −−−− decision variables −−−−−−−−−
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)
X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%
h = 0.5;
f_cnt = @(x,u) quad.f(x,u); %continous model
f_discrete = @(x,u) RK4(x,u,h,f_cnt); %discretized model

Q = 10;
Q2 = 20;
R = 5;

opti.minimize(...
    R*(U(1,:))*(U(1,:))' + ...
    R*(U(2,:))*(U(2,:))' + ...
    R*(U(3,:))*(U(3,:))' + ...
    R*(U(4,:))*(U(4,:))' + ...
    Q2*(X(6,:)-REF(4))*(X(6,:)-REF(4))' + ...
    Q2*(X(10,:)-REF(1))*(X(10,:)-REF(1))' + ...
    Q2*(X(11,:)-REF(2))*(X(11,:)-REF(2))'+...
    Q2*(X(12,:)-REF(3))*(X(12,:)-REF(3))'+...
    Q*X(1,:)*X(1,:)'+...
    Q*X(2,:)*X(2,:)'+...
    Q*X(3,:)*X(3,:)'+...
    Q*X(4,:)*X(4,:)'+...
    Q*X(5,:)*X(5,:)'+...
    Q*X(7,:)*X(7,:)'+...
    Q*X(8,:)*X(8,:)'+...
    Q*X(9,:)*X(9,:)'); 

opti.subject_to(0<=U<=1.5);
for i=1:1:N
    opti.subject_to(X(:,i+1)==f_discrete(X(:,i),U(:,i)));
end
opti.subject_to(X(:,1) == X0);
%%%%%%%%%%%%%%%%%%%%%%%%
ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);

end

function u = eval_ctrl(x, ref, opti, X0, REF, ~, U)

% −−−− Set the initial state and reference −−−−
opti.set_value(X0, x);
opti.set_value(REF, ref);
% −−−− Setup solver NLP −−−−−−
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);
% −−−− Solve the optimization problem −−−−
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));
% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));

end