quad = Quad();
Tf = 1.0;
x0 = zeros(12,1);
u = [1;0.5;1;1];

sim = ode45(@(t,x) quad.f(x,u), [0, Tf], x0);
quad.plot(sim, u);