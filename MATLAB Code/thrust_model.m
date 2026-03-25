function [T, P_shaft] = thrust_model(ac, V, phase, T_required, C)
% THRUST_MODEL  Computes thrust [N] and shaft power [W].
%   Inputs:  ac          — aircraft struct
%            V           — airspeed [m/s]
%            phase       — 'hover' | 'transition' | 'climb' | 'cruise' | 'descent'
%            T_required  — required thrust [N] (ignored for hover/transition)
%            C           — constants struct

g   = C.phys.g;
rho = C.phys.rho;
W   = ac.MTOW * g;
FM  = 0.7;

switch lower(phase)

    case 'hover'
        if ~ac.has_hover
            error('thrust_model: eSTOL does not have a hover phase.');
        end
        T       = W;
        v_i     = sqrt(T / (2 * rho * ac.A_disk));
        P_shaft = (T * v_i) / (FM * ac.eta_motor);

    case 'transition'
        if ~ac.has_transition
            error('thrust_model: eSTOL does not have a transition phase.');
        end
        V_trans  = 0.80 * ac.V_best_range;
        blend    = min(1.0, max(0.0, V / V_trans));
        CL_trans = ac.CLalpha * deg2rad(3.0);
        L_wing   = min(0.5 * rho * V^2 * ac.S_wing * CL_trans, W);
        W_rotor  = max(0, W - L_wing);
        T_rotor  = W_rotor;
        T        = T_rotor * sin(blend * pi/2);
        v_i      = sqrt(max(T_rotor / (2 * rho * ac.A_disk), 0));
        V_eff    = sqrt(v_i^2 + (V/2)^2);
        P_shaft  = (T_rotor * V_eff) / (FM * ac.eta_motor);

    case {'climb', 'cruise', 'descent'}
        % T_required is the actual thrust needed — computed by mission_energy.m
        T       = T_required;
        V_eff   = max(V, 1.0);
        P_shaft = T * V_eff / (ac.eta_motor * ac.eta_prop);

    otherwise
        error('thrust_model: unknown phase "%s".', phase);
end

end
