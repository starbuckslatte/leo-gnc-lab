function [tau, fBody] = aerodynamic(rEci, vEci, q, rho, sc)
%TORQUE.AERODYNAMIC Aerodynamic drag torque [N m] and force [N] in body frame.
%   Single flat plate of constant area facing the flow:
%       F = -0.5 rho Cd A |v_rel| v_rel,   tau = cpOffset x F
%   v_rel = v - w_earth x r (co-rotating atmosphere).
%   sc fields: dragArea [m^2], Cd, cpOffset [m] (CoP - CoM, body).
W_EARTH = 7.292115146706979e-5;
vRel = vEci(:) - cross([0; 0; W_EARTH], rEci(:));
vb = att.toDCM(q) * vRel;
fBody = -0.5 * rho * sc.Cd * sc.dragArea * norm(vb) * vb;
tau = cross(sc.cpOffset(:), fBody);
end
