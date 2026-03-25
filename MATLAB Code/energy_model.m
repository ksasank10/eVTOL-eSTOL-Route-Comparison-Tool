function energy = energy_model(ac, t_vec, x_mat, phase, T_required, C)
% ENERGY_MODEL  Computes energy over a simulated flight phase.
%   Inputs:  ac          — aircraft struct
%            t_vec       — time vector [s], N x 1
%            x_mat       — state matrix [V, gamma, h, x_pos], N x 4
%            phase       — flight phase string
%            T_required  — required thrust [N] (scalar, from mission_energy.m)
%            C           — constants struct

N     = length(t_vec);
P_vec = zeros(N, 1);

for i = 1:N
    V = x_mat(i, 1);
    if strcmp(phase, 'hover') || strcmp(phase, 'transition')
        V = max(V, 0);
    else
        V = max(V, 1.0);
    end
    [~, P_vec(i)] = thrust_model(ac, V, phase, T_required, C);
end

energy.P_vec        = P_vec;
energy.E_total_J    = trapz(t_vec, P_vec);
energy.E_total_kWh  = energy.E_total_J / 3.6e6;
energy.peak_power_W = max(P_vec);
energy.t_vec        = t_vec;

end
