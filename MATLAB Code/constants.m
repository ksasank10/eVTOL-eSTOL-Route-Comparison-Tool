function C = constants()
% CONSTANTS  Fixed physical constants and simulation settings.
%
%   These values never come from the user. They are physical laws
%   (gravity, air density) and numerical solver settings. Everything
%   aircraft-specific or mission-specific comes from the user via the App.
%
%   Output:
%     C  - struct with fields:
%            .phys   physical constants
%            .sim    ode45 solver settings
%
% Physical constants
C.phys.g      = 9.81;     % gravitational acceleration [m/s^2]
C.phys.rho    = 1.225;    % air density, sea level ISA [kg/m^3]
C.phys.rho_h  = @(h) 1.225 * exp(-h / 8500);
%   Simple exponential atmosphere model.
%   rho at altitude h [m] is used in climb/cruise phases.

% Simulation / solver settings
C.sim.tol_rel  = 1e-6;   % ode45 relative tolerance
C.sim.tol_abs  = 1e-8;   % ode45 absolute tolerance
C.sim.t_max    = 3600;   % max time per flight phase [s] (1 hr safety cap)
C.sim.dt_out   = 0.5;    % output timestep for post-processing [s]
%   ode45 uses adaptive internal steps — dt_out only controls
%   the resolution of the returned time vector for plotting.

end
