function dxdt = eom_3dof(t, x, ac, phase, gamma_target, T_required, C)
% EOM_3DOF  Longitudinal 3-DOF equations of motion. Called by ode45.
%   State: x = [V, gamma, h, x_pos]
%   Thrust controller maintains gamma_target via proportional correction on T_required.

V     = max(x(1), 1.0);
gamma = x(2);
h     = max(x(3), 0.0);

rho = C.phys.rho_h(h);
g   = C.phys.g;
m   = ac.MTOW;

% Angle of attack from quasi-static trim
q      = 0.5 * rho * V^2;
CL_req = (m * g * cos(gamma)) / max(q * ac.S_wing, 1e-6);
CL_req = max(-0.5, min(1.8, CL_req));

alpha0_rad = deg2rad(-2.0);
alpha_rad  = CL_req / ac.CLalpha + alpha0_rad;
alpha_rad  = max(deg2rad(-5), min(deg2rad(14), alpha_rad));

[CL, CD, ~] = aero_lookup(rad2deg(alpha_rad), ac, 'polar');
L = q * ac.S_wing * CL;
D = q * ac.S_wing * CD;

% Proportional gamma controller — adjusts T_required to hold gamma_target
Kp = 2.0;
T  = T_required + Kp * (gamma_target - gamma) * m * g;
T  = max(0, min(T, 4 * m * g));

dVdt     = (T * cos(alpha_rad) - D) / m  -  g * sin(gamma);
dgammadt = (T * sin(alpha_rad) + L - m * g * cos(gamma)) / (m * V);
dhdt     = V * sin(gamma);
dxdt_pos = V * cos(gamma);

dxdt = [dVdt; dgammadt; dhdt; dxdt_pos];

end
