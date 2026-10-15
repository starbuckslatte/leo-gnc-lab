function x = rk4Step(f, t, x, dt)
%NUM.RK4STEP One classical fourth-order Runge-Kutta step of xdot = f(t, x).
k1 = f(t,          x);
k2 = f(t + dt/2,   x + dt/2*k1);
k3 = f(t + dt/2,   x + dt/2*k2);
k4 = f(t + dt,     x + dt*k3);
x  = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
