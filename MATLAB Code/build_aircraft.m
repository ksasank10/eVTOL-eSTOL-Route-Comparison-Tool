function ac = build_aircraft(inputs)
% This function validates the user input parameters and uses them to build 
% the aircraft struct. This is the function the App calls when the user 
% hits "Run Analysis".

ac = inputs;

% Default CL_blown to 1.0 if not provided (eVTOL doesn't use it)
if ~isfield(ac, 'CL_blown')
    ac.CL_blown = 1.0;
end

% Type
if ~ismember(ac.type, {'evtol', 'estol'})
    error('build_aircraft: type must be ''evtol'' or ''estol''. Got: %s', ac.type);
end

% Mass
if ac.MTOW <= 0 || ac.MTOW > 15000
    error('build_aircraft: MTOW must be between 0 and 15,000 kg. Got: %.1f', ac.MTOW);
end
if ac.m_empty <= 0 || ac.m_empty >= ac.MTOW
    error('build_aircraft: Empty mass must be positive and less than MTOW (%.0f kg). Got: %.1f', ac.MTOW, ac.m_empty);
end

% Wing geometry 
if ac.S_wing <= 0 || ac.S_wing > 300
    error('build_aircraft: Wing area must be between 0 and 300 m^2. Got: %.1f', ac.S_wing);
end
if ac.AR < 2 || ac.AR > 25
    error('build_aircraft: Aspect ratio must be between 2 and 25. Got: %.1f', ac.AR);
end
if ac.e_oswald <= 0 || ac.e_oswald > 1.0
    error('build_aircraft: Oswald efficiency must be between 0 and 1. Got: %.2f', ac.e_oswald);
end
if ac.CD0 < 0.005 || ac.CD0 > 0.15
    error('build_aircraft: CD0 must be between 0.005 and 0.15. Got: %.4f', ac.CD0);
end

% Propulsion 
if ac.n_rotors < 1 || ac.n_rotors > 20 || floor(ac.n_rotors) ~= ac.n_rotors
    error('build_aircraft: Number of rotors must be an integer between 1 and 20. Got: %.1f', ac.n_rotors);
end
if ac.A_rotor <= 0 || ac.A_rotor > 50
    error('build_aircraft: Rotor disk area must be between 0 and 50 m^2. Got: %.2f', ac.A_rotor);
end
if ac.eta_motor <= 0.5 || ac.eta_motor > 1.0
    error('build_aircraft: Motor efficiency must be between 0.5 and 1.0. Got: %.3f', ac.eta_motor);
end
if ac.eta_prop <= 0.3 || ac.eta_prop > 1.0
    error('build_aircraft: Prop efficiency must be between 0.3 and 1.0. Got: %.3f', ac.eta_prop);
end

% Battery 
if ac.battery_kWh <= 0 || ac.battery_kWh > 5000
    error('build_aircraft: Battery capacity must be between 0 and 5000 kWh. Got: %.1f', ac.battery_kWh);
end

% eSTOL blown lift 
if ac.CL_blown < 1.0 || ac.CL_blown > 6.0
    error('build_aircraft: Blown lift multiplier must be between 1.0 and 6.0. Got: %.2f', ac.CL_blown);
end

% Computing derived aerodynamic quantities

% Induced drag factor
%   CD = CD0 + k * CL^2   (parabolic drag polar)
%   k = 1 / (pi * AR * e)
ac.k_drag = 1 / (pi * ac.AR * ac.e_oswald);

% Wing geometry
ac.b_span = sqrt(ac.AR * ac.S_wing);     
ac.c_mac  = ac.S_wing / ac.b_span;       

% Lift curve slope — Helmbold equation
%   Accounts for finite wing effects on the 2*pi/rad thin airfoil slope.
%   a0 = 2*pi [1/rad]

a0 = 2 * pi;
ac.CLalpha = a0 / (1 + (a0 / (pi * ac.AR)));   % [1/rad]

% Peak lift-to-drag ratio (closed form from parabolic polar)
%   Occurs at CL* = sqrt(CD0 / k), where dCD/dCL = 0
%   (L/D)_max = CL* / (2 * CD0)
CL_star   = sqrt(ac.CD0 / ac.k_drag);
ac.peak_LD = CL_star / (2 * ac.CD0);

% Derived propulsion quantities

% Total disk area
ac.A_disk = ac.n_rotors * ac.A_rotor;    % [m^2]

% Disk loading (lower = more hover-efficient, quieter) [N/m^2]

C = constants();
ac.disk_loading = (ac.MTOW * C.phys.g) / ac.A_disk;   % [N/m^2]

%  Step 5 — Compute derived mass / energy quantities


% Wing loading
ac.wing_loading = ac.MTOW / ac.S_wing;   % [kg/m^2]

% Battery energy in SI units
ac.battery_J = ac.battery_kWh * 3.6e6;  % [J]

%  Step 6 — Compute characteristic speeds (analytic, sea level ISA)

rho = C.phys.rho;

% Speed for best range (minimum drag speed)
%   At (L/D)_max: V_br = sqrt(2*W / (rho * S * CL*))
%   where W = MTOW * g, CL* = sqrt(CD0/k)
ac.V_best_range = sqrt((2 * ac.MTOW * C.phys.g) / ...
                       (rho * ac.S_wing * CL_star));     % [m/s]

% Minimum power speed (for hover-to-cruise transition reference)
%   V_mp = V_br / 3^(1/4) ≈ 0.76 * V_br
ac.V_min_power  = ac.V_best_range * (3^(-0.25));         % [m/s]

% Stall speed (rough estimate, CL_max = 1.5 for clean config)
CL_max_clean = 1.5;
ac.V_stall = sqrt((2 * ac.MTOW * C.phys.g) / ...
                  (rho * ac.S_wing * CL_max_clean));     % [m/s]


ac.has_hover      = strcmp(ac.type, 'evtol');
ac.has_transition = strcmp(ac.type, 'evtol');
ac.has_blown_lift = strcmp(ac.type, 'estol');

if nargout == 0
    fprintf('\n=== %s — build_aircraft summary ===\n', upper(ac.name));
    fprintf('%-28s %10.1f kg\n',    'MTOW:',          ac.MTOW);
    fprintf('%-28s %10.1f m^2\n',   'Wing area:',      ac.S_wing);
    fprintf('%-28s %10.1f\n',       'Aspect ratio:',   ac.AR);
    fprintf('%-28s %10.1f kg/m^2\n','Wing loading:',   ac.wing_loading);
    fprintf('%-28s %10.4f\n',       'k_drag:',         ac.k_drag);
    fprintf('%-28s %10.1f\n',       'Peak L/D:',       ac.peak_LD);
    fprintf('%-28s %10.2f 1/rad\n', 'CLalpha:',        ac.CLalpha);
    fprintf('%-28s %10.1f m^2\n',   'Total disk area:', ac.A_disk);
    fprintf('%-28s %10.1f N/m^2\n', 'Disk loading:',   ac.disk_loading);
    fprintf('%-28s %10.1f m/s\n',   'V_best_range:',   ac.V_best_range);
    fprintf('%-28s %10.1f m/s\n',   'V_stall:',        ac.V_stall);
    fprintf('%-28s %10.1f kWh\n',   'Battery:',        ac.battery_kWh);
    fprintf('%-28s %10s\n',         'Has hover phase:', mat2str(ac.has_hover));
    fprintf('%-28s %10s\n',         'Has blown lift:',  mat2str(ac.has_blown_lift));
    fprintf('\n');
end

end
