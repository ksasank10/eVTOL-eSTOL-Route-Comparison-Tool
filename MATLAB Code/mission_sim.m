function metrics = mission_sim(ac, route, C)
% MISSION_SIM  Top-level mission simulation. Called by the App on "Run Analysis".
%
%   Runs the full route for one aircraft and returns a complete metrics
%   struct ready for scoring_framework.m. Wraps mission_energy.m and
%   transition_opt.m into a single callable function.


%  Tier 1 metrics: direct from aircraft struct, no simulation
metrics.peak_LD             = ac.peak_LD;
metrics.wing_loading        = ac.wing_loading;
metrics.disk_loading        = ac.disk_loading;
metrics.n_independent_lift  = ac.n_rotors;
metrics.V_best_range        = ac.V_best_range;
metrics.landing_footprint_m = route.estol_offset_m * double(~ac.has_hover);

%   eVTOL: footprint = 0 (lands anywhere)
%   eSTOL: footprint = estol_offset_m (distance to nearest runway)

%  Tier 2: run mission energy simulation
energy_metrics = mission_energy(ac, route, C);

% Copy all energy metrics into output struct
fields = fieldnames(energy_metrics);
for i = 1:length(fields)
    metrics.(fields{i}) = energy_metrics.(fields{i});
end

%  Tier 2: transition optimization (eVTOL only)
if ac.has_transition
    tr = transition_opt(ac, C);
    metrics.transition_penalty_pct = tr.penalty_pct;
    metrics.transition_result      = tr;   % full result for plotting
else
    metrics.transition_penalty_pct = 0;
    metrics.transition_result      = [];
end


%  Feasibility check: if route is not completable, zero key metrics

if ~metrics.feasible
    warning('mission_sim: %s cannot complete this route above reserve. SoC=%.1f%%', ...
        ac.name, metrics.soc_at_landing * 100);
end

end
