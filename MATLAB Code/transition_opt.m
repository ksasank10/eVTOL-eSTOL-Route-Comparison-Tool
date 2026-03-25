function result = transition_opt(ac, C)
% TRANSITION_OPT  Finds the minimum energy transition profile for eVTOL.
%   This function solves for the optimal speed schedule using fmincon,
%   then compares it against a naive linear acceleration (constant dV/dt).

if ~ac.has_transition
    error('transition_opt: only valid for eVTOL (ac.has_transition = true)');
end

% Transition parameters
V_start = 2.0;                      % just above hover [m/s]
V_end   = 0.80 * ac.V_best_range;  % transition complete [m/s]
T_total = 90;                       % total transition time [s]
N       = 45;                       % number of time nodes

t_vec = linspace(0, T_total, N)';
dt    = T_total / (N - 1);
dV_total = V_end - V_start;

% Naive transition: linear acceleration (constant dV/dt)

V_naive = linspace(V_start, V_end, N)';
P_naive = zeros(N, 1);
for i = 1:N
    [~, P_naive(i)] = thrust_model(ac, V_naive(i), 'transition', 0, C);
end
E_naive_J   = trapz(t_vec, P_naive);
E_naive_kWh = E_naive_J / 3.6e6;


% Optimal transition: fmincon minimizes total energy
%
%   Decision variable x = dV 
%   V(i) = V_start + cumsum(x)
%   Constraint: sum(x) = dV_total (must reach V_end)


% Objective: total energy for a given dV schedule
    function E = energy_cost(dV)
        V_prof = V_start + [0; cumsum(dV)];
        P_prof = zeros(N, 1);
        for k = 1:N
            [~, P_prof(k)] = thrust_model(ac, V_prof(k), 'transition', 0, C);
        end
        E = trapz(t_vec, P_prof);
    end

% Initial guess: linear (naive) increments
dV0 = ones(N-1, 1) * dV_total / (N-1);

% Bounds: each increment >= 0 (monotonic), <= 2x average (no huge jumps)
lb = zeros(N-1, 1);
ub = ones(N-1, 1) * 2 * dV_total / (N-1);

% Equality constraint: sum of increments = total speed change
Aeq = ones(1, N-1);
beq = dV_total;

options = optimoptions('fmincon', ...
    'Display',          'off', ...
    'Algorithm',        'sqp', ...
    'MaxIterations',    500, ...
    'OptimalityTol',    1e-4, ...
    'ConstraintTol',    1e-4);

[dV_opt, E_opt_J] = fmincon(@energy_cost, dV0, [], [], Aeq, beq, lb, ub, [], options);

V_optimal   = V_start + [0; cumsum(dV_opt)];
E_opt_kWh   = E_opt_J / 3.6e6;

% Power profile for optimal
P_optimal = zeros(N, 1);
for i = 1:N
    [~, P_optimal(i)] = thrust_model(ac, V_optimal(i), 'transition', 0, C);
end

% Results
result.E_naive_kWh     = E_naive_kWh;
result.E_optimal_kWh   = E_opt_kWh;
result.penalty_pct     = (E_naive_kWh - E_opt_kWh) / E_opt_kWh * 100;
result.V_naive         = V_naive;
result.V_optimal       = V_optimal;
result.t_vec           = t_vec;
result.P_naive         = P_naive;
result.P_optimal       = P_optimal;
result.V_transition_ms = V_end;
result.t_transition_s  = T_total;

end
