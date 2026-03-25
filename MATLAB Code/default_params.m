function [evtol_defaults, estol_defaults, route_defaults] = default_params()
% DEFAULT_PARAMS  Returns default input values that pre-populate the App UI.

%  eVTOL defaults
evtol_defaults.type        = 'evtol';    
evtol_defaults.name        = 'eVTOL';

% Mass
evtol_defaults.MTOW        = 2177;       
evtol_defaults.m_empty     = 1300;      

% Wing
evtol_defaults.S_wing      = 10.7;       
evtol_defaults.AR          = 8.5;        
evtol_defaults.e_oswald    = 0.80;       
evtol_defaults.CD0         = 0.025;      

% Propulsion
evtol_defaults.n_rotors    = 6;          
evtol_defaults.A_rotor     = 3.8;        
evtol_defaults.eta_motor   = 0.95;      
evtol_defaults.eta_prop    = 0.82;       
% Battery
evtol_defaults.battery_kWh = 200;      

%  eSTOL defaults
estol_defaults.type        = 'estol';    
estol_defaults.name        = 'eSTOL';

% Mass
estol_defaults.MTOW        = 3175;      
estol_defaults.m_empty     = 2100;       

% Wing
estol_defaults.S_wing      = 30.0;       
estol_defaults.AR          = 12.0;       
estol_defaults.e_oswald    = 0.88;       
estol_defaults.CD0         = 0.032;      
estol_defaults.CL_blown    = 1.35;       

% Propulsion
estol_defaults.n_rotors    = 8;          
estol_defaults.A_rotor     = 0.80;       
estol_defaults.eta_motor   = 0.95;       
estol_defaults.eta_prop    = 0.87;       

% Battery
estol_defaults.battery_kWh = 150;        % [kWh]

% Mission defaults -> These are what the user sets per analysis run.

route_defaults.distance_km      = 50;    % ground range [km]
route_defaults.cruise_alt_m     = 300;   % cruise altitude AGL [m]
route_defaults.cruise_speed_ms  = 55;    % target cruise speed [m/s] (~200 km/h)
route_defaults.payload_kg       = 400;   % payload: ~4 passengers [kg]
route_defaults.climb_rate_ms    = 3.0;   % climb rate [m/s]
route_defaults.descent_rate_ms  = 2.0;   % descent rate [m/s]
route_defaults.hover_time_s     = 30;    % hover duration at takeoff + landing [s each]
route_defaults.battery_reserve  = 0.20;  % minimum SoC fraction [0-1]
route_defaults.estol_offset_km  = 2.0;   % extra distance eSTOL flies to reach runway [km]


end
