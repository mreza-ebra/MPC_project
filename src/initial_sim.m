quad = Quad();
Tf = 1.0;
x0 = zeros(12,1);
u = [0.7007;0.7007;0.7007;0.7007];

sim = ode45(@(t,x) quad.f(x,u), [0, Tf], x0);
quad.plot(sim, u);