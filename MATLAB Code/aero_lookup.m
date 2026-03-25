function [CL, CD, Cm] = aero_lookup(alpha_deg, ac, mode)
% AERO_LOOKUP  Returns CL, CD, Cm at a given angle of attack.
%   Inputs:  alpha_deg — AoA [deg], scalar or vector
%            ac        — aircraft struct from build_aircraft.m
%            mode      — 'polar' (default) or 'avl'
%   Outputs: CL, CD, Cm — aerodynamic coefficients [-]

if nargin < 3
    mode = 'polar';
end

switch lower(mode)

    case 'polar'
        [CL, CD, Cm] = polar_model(alpha_deg, ac);

    case 'avl'
        if ~isfield(ac, 'avl_table') || ~isfile(ac.avl_table)
            warning('aero_lookup: AVL table not found. Using polar model.');
            [CL, CD, Cm] = polar_model(alpha_deg, ac);
            return
        end
        data    = load(ac.avl_table, 'alpha_table', 'CL_table', 'CD_table', 'Cm_table');
        alpha_c = max(data.alpha_table(1), min(data.alpha_table(end), alpha_deg));
        CL      = interp1(data.alpha_table, data.CL_table, alpha_c, 'linear');
        CD      = interp1(data.alpha_table, data.CD_table, alpha_c, 'linear');
        Cm      = interp1(data.alpha_table, data.Cm_table, alpha_c, 'linear');

    otherwise
        error('aero_lookup: unknown mode "%s". Use ''polar'' or ''avl''.', mode);
end

end

function [CL, CD, Cm] = polar_model(alpha_deg, ac)
% Parabolic drag polar + linear pitch moment.
% CL  = CLalpha * (alpha - alpha0)     [linear lift curve]
% CD  = CD0 + k * CL^2                [parabolic polar]
% Cm  = Cmalpha * (alpha - alpha_trim) [linear pitch moment]
% No blown lift correction — CL_blown belongs in thrust_model.m only.

    alpha_rad  = deg2rad(alpha_deg);
    alpha0_rad = deg2rad(-2.0);

    CL = ac.CLalpha .* (alpha_rad - alpha0_rad);
    CL = max(-0.8, min(1.8, CL));

    CD = ac.CD0 + ac.k_drag .* CL.^2;

    Cmalpha    = -0.5 * ac.CLalpha / pi;
    alpha_trim = deg2rad(2.0);
    Cm         = Cmalpha .* (alpha_rad - alpha_trim);

end
