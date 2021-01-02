quad = Quad();
[xs,us] = quad.trim()        % Compute steady−state for which 0 = f(xs,us)
sys = quad.linearize(xs, us) % Linearize the nonlinear model