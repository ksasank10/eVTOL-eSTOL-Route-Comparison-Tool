function metrics = mission_energy(ac, route, C)
% MISSION_ENERGY  Simulates a full route and returns energy metrics.
%
%   Flight phases:
%     eVTOL: hover -> transition -> climb -> cruise -> descent -> hover
%     eSTOL: climb -> cruise -> descent
%
%   eSTOL effective distance = route.distance_m + route.estol_offset_m
%
%   Returns metrics struct for scoring_framework.m.

g    = C.phys.g;
rho  = C.phys.rho;
opts = odeset('RelTol', C.sim.tol_rel, 'AbsTol', C.sim.tol_abs);

if ac.has_hover
    eff_dist_m = route.distance_m;
else
    eff_dist_m = route.distance_m + route.estol_offset_m;
end

V_cr        = route.cruise_speed_ms;
h_cr        = route.cruise_alt_m;
gamma_climb = asin(min(0.25, route.climb_rate_ms  / V_cr));
gamma_desc  = asin(min(0.25, route.descent_rate_ms / V_cr));

% Pre-compute required thrust for each phase
% T = D in cruise (level, constant speed)
% T = D + W*sin(gamma) in climb
% T = D - W*sin(gamma) in descent (reduced power, still positive)

q_cr   = 0.5 * rho * V_cr^2;
CL_cr  = (ac.MTOW * g) / max(q_cr * ac.S_wing, 1e-6);
CL_cr  = max(0.1, min(1.8, CL_cr));
alpha_cr = rad2deg(CL_cr / ac.CLalpha + deg2rad(-2.0));
[~, CD_cr, ~] = aero_lookup(alpha_cr, ac, 'polar');
D_cr   = q_cr * ac.S_wing * CD_cr;

T_cruise = D_cr;
T_climb  = D_cr + ac.MTOW * g * sin(gamma_climb);
T_desc   = max(0, D_cr - ac.MTOW * g * sin(gamma_desc) * 0.5);

% Accumulators
t_all = []; P_all = []; h_all = []; x_all = [];
t_offset = 0;
phase_energy = struct();

%  Takeoff hover (eVTOL only)

if ac.has_hover
    t_h  = linspace(0, route.hover_time_s, 50)';
    x_h  = zeros(length(t_h), 4);
    e_h  = energy_model(ac, t_h, x_h, 'hover', 0, C);
    phase_energy.hover_kWh = e_h.E_total_kWh;
    t_all = [t_all; t_h + t_offset];
    P_all = [P_all; e_h.P_vec];
    h_all = [h_all; x_h(:,3)];
    x_all = [x_all; x_h(:,4)];
    t_offset = t_offset + route.hover_time_s;
else
    phase_energy.hover_kWh = 0;
end

%  Transition (eVTOL only)

if ac.has_transition
    V_trans_end = 0.80 * ac.V_best_range;
    t_tr  = linspace(0, 60, 60)';
    V_tr  = linspace(1, V_trans_end, 60)';
    x_tr  = [V_tr, zeros(60,1), ones(60,1)*10, zeros(60,1)];
    e_tr  = energy_model(ac, t_tr, x_tr, 'transition', 0, C);
    phase_energy.transition_kWh = e_tr.E_total_kWh;
    t_all = [t_all; t_tr + t_offset];
    P_all = [P_all; e_tr.P_vec];
    h_all = [h_all; x_tr(:,3)];
    x_all = [x_all; x_tr(:,4)];
    t_offset = t_offset + 60;
else
    phase_energy.transition_kWh = 0;
end

%  Climb (both aircraft)

x0_cl   = [V_cr; gamma_climb; 0; 0];
opts_cl = odeset(opts, 'Events', @(t,x) alt_event(t, x, h_cr));
f_cl    = @(t,x) eom_3dof(t, x, ac, 'climb', gamma_climb, T_climb, C);
[t_cl, x_cl] = ode45(f_cl, [0 C.sim.t_max], x0_cl, opts_cl);
e_cl  = energy_model(ac, t_cl, x_cl, 'climb', T_climb, C);
phase_energy.climb_kWh = e_cl.E_total_kWh;
range_after_climb = x_cl(end, 4);
t_all = [t_all; t_cl + t_offset];
P_all = [P_all; e_cl.P_vec];
h_all = [h_all; x_cl(:,3)];
x_all = [x_all; x_cl(:,4)];
t_offset = t_offset + t_cl(end);

%  Cruise (both aircraft)

range_desc_approx = h_cr / tan(gamma_desc);
range_cr_m = max(eff_dist_m - range_after_climb - range_desc_approx, 1000);
t_cruise_s = range_cr_m / V_cr;

t_cr  = linspace(0, t_cruise_s, max(50, round(t_cruise_s)))';
x_cr  = [repmat(V_cr, length(t_cr), 1), zeros(length(t_cr),1), ...
          repmat(h_cr, length(t_cr), 1), ...
          range_after_climb + V_cr * t_cr];
e_cr  = energy_model(ac, t_cr, x_cr, 'cruise', T_cruise, C);
phase_energy.cruise_kWh = e_cr.E_total_kWh;
t_all = [t_all; t_cr + t_offset];
P_all = [P_all; e_cr.P_vec];
h_all = [h_all; x_cr(:,3)];
x_all = [x_all; x_cr(:,4)];
t_offset = t_offset + t_cruise_s;

%  Descent (both aircraft)

x0_ds   = [V_cr; -gamma_desc; h_cr; x_cr(end,4)];
opts_ds = odeset(opts, 'Events', @(t,x) alt_event_desc(t, x));
f_ds    = @(t,x) eom_3dof(t, x, ac, 'descent', -gamma_desc, T_desc, C);
[t_ds, x_ds] = ode45(f_ds, [0 C.sim.t_max], x0_ds, opts_ds);
e_ds  = energy_model(ac, t_ds, x_ds, 'descent', T_desc, C);
phase_energy.descent_kWh = e_ds.E_total_kWh;
t_all = [t_all; t_ds + t_offset];
P_all = [P_all; e_ds.P_vec];
h_all = [h_all; x_ds(:,3)];
x_all = [x_all; x_ds(:,4)];
t_offset = t_offset + t_ds(end);

%  Landing hover (eVTOL only)

if ac.has_hover
    t_h2  = linspace(0, route.hover_time_s, 50)';
    x_h2  = zeros(length(t_h2), 4);
    e_h2  = energy_model(ac, t_h2, x_h2, 'hover', 0, C);
    phase_energy.landing_hover_kWh = e_h2.E_total_kWh;
    t_all = [t_all; t_h2 + t_offset];
    P_all = [P_all; e_h2.P_vec];
    h_all = [h_all; x_h2(:,3)];
    x_all = [x_all; x_h2(:,4)];
else
    phase_energy.landing_hover_kWh = 0;
end

%  Aggregate metrics

E_total_kWh = sum(struct2array(phase_energy));
E_hover_kWh = phase_energy.hover_kWh + phase_energy.landing_hover_kWh;

metrics.energy_total_kWh      = E_total_kWh;
metrics.energy_per_km         = E_total_kWh / (eff_dist_m / 1e3);
metrics.peak_power_kW         = max(P_all) / 1e3;
metrics.hover_energy_fraction = E_hover_kWh / max(E_total_kWh, 1e-6);
metrics.soc_at_landing        = (ac.battery_kWh - E_total_kWh) / ac.battery_kWh;
metrics.mission_time_s        = t_all(end);
metrics.feasible              = metrics.soc_at_landing >= route.battery_reserve;
metrics.phase_energy_kWh      = phase_energy;
metrics.eff_dist_km           = eff_dist_m / 1e3;
metrics.t_full                = t_all;
metrics.P_full                = P_all;
metrics.h_full                = h_all;
metrics.x_full                = x_all;

end

function [v, ist, dir] = alt_event(~, x, h_target)
    v = x(3) - h_target; ist = 1; dir = 1;
end

function [v, ist, dir] = alt_event_desc(~, x)
    v = x(3); ist = 1; dir = -1;
end
