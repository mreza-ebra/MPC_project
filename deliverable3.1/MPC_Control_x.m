classdef MPC_Control_x < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 15; 
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
        
      A = mpc.A;
      B = mpc.B;
      C = mpc.C;
      D = mpc.D;
      
      Q = 10 * eye(n);
      R = 7;
      
      % Constraints
      % u in U = { u | Mu <= m }
      M = [1;-1]; 
      m = [0.3; 0.3];
      % x in X = { x | Fx <= f }
      F = [0 1 0 0; 0 -1 0 0];
      f = [0.035; 0.035]; %radians
      
      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
      
      % Compute LQR controller for unconstrained system
      [K,Qf,~] = dlqr(A,B,Q,R);
      % MATLAB defines K as -K, so invert its signal
      K = -K; 

      % Compute maximal invariant set
      Xf = polytope([F; M*K],[f;m]);
      Acl = [A+B*K];
      while 1
          prevXf = Xf;
          [T,t] = double(Xf);
          preXf = polytope(T*Acl,t);
          Xf = intersect(Xf, preXf);
          if isequal(prevXf, Xf)
              break
          end
      end
      [Ff,ff] = double(Xf);
      
      %Plot of terminal invariant set for each of the dimensions
      
      figure(3)
      hold on; grid on;
      
      
      subplot(3,1,1)
      hold on; grid on;
      title('Terminal invariant set for the system x');
      Xf.projection(1:2).plot();
      xlabel('$\dot{\beta}[rad/s]$','interpreter','latex');
      ylabel('$\beta[rad]$','interpreter','latex');
      
      subplot(3,1,2)
      hold on; grid on;
      Xf.projection(2:3).plot();
      xlabel('$\beta[rad]$','interpreter','latex');
      ylabel('$\dot{x}[m/s]$','interpreter','latex');
      
      subplot(3,1,3)
      hold on; grid on;
      Xf.projection(3:4).plot();
      xlabel('$\dot{x}[m/s]$','interpreter','latex');
      ylabel('$x[m]$','interpreter','latex');

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      con = (x(:,2) == A*x(:,1) + B*u(:,1)) + (M*u(:,1) <= m);
      obj = u(:,1)'*R*u(:,1);
      for i = 2:N-1
        con = con + (x(:,i+1) == A*x(:,i) + B*u(:,i));
        con = con + (F*x(:,i) <= f) + (M*u(:,i) <= m);
        obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
      end
      con = con + (Ff*x(:,N) <= ff); % terminal constraint
      obj = obj + x(:,N)'*Qf*x(:,N); % terminal weight
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      obj = 0;
      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
