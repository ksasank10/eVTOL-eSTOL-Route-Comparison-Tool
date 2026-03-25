function scores = scoring_framework(m_e, m_s, metric_weights)
% SCORING_FRAMEWORK  Computes weighted scores for eVTOL vs eSTOL.
%
%   Uses direct per-metric weighting (weights sum to 1.0 globally).
%   Categories are display labels only — all weighting happens at the
%   individual metric level, avoiding the hierarchical averaging problem
%   where category size distorts effective metric influence.
%
%   Effective metric influence = metric_weight (explicit, not w_cat/n_metrics)
%
%   Default metric weights (sum = 1.0):
%     energy_per_km    0.22  — primary efficiency metric
%     peak_power_kW    0.13  — motor/battery sizing driver
%     soc_at_landing   0.12  — safety margin at destination
%     mission_time_s   0.08  — door-to-door speed
%     peak_LD          0.12  — core aerodynamic quality
%     V_best_range     0.08  — urban ops flexibility
%     eff_dist_km      0.13  — direct infrastructure penalty
%     energy_total_kWh 0.12  — absolute battery draw
%
%   Reference ranges (physically justified UAM bounds):
%     energy_per_km:   [0.3, 3.0]  kWh/km
%     peak_power_kW:   [50,  800]  kW
%     soc_at_landing:  [0.20, 1.0] fraction
%     mission_time_s:  [300, 2400] s
%     peak_LD:         [5,   25]   -
%     V_best_range:    [20,  80]   m/s
%     eff_dist_km:     [d, 1.25d]  km  (dynamic, route-dependent)
%     energy_total_kWh:[5,  200]   kWh


% Default per-metric weights — sum to 1.0
if nargin < 3
    metric_weights = struct(...
        'energy_per_km',    0.22, ...
        'peak_power_kW',    0.13, ...
        'soc_at_landing',   0.12, ...
        'mission_time_s',   0.08, ...
        'peak_LD',          0.12, ...
        'V_best_range',     0.08, ...
        'eff_dist_km',      0.13, ...
        'energy_total_kWh', 0.12  ...
    );
end

% Validate weights sum to 1
w_fields = fieldnames(metric_weights);
w_total  = 0;
for i = 1:length(w_fields)
    w_total = w_total + metric_weights.(w_fields{i});
end
if abs(w_total - 1.0) > 0.01
    error('scoring_framework: metric weights must sum to 1.0. Got %.3f', w_total);
end

% Feasibility gate
if ~m_e.feasible
    warning('scoring_framework: eVTOL cannot complete route. Score = 0.');
end
if ~m_s.feasible
    warning('scoring_framework: eSTOL cannot complete route. Score = 0.');
end

% Metrics definition
% {field, direction, ref_min, ref_max, category_label}

base_dist   = m_e.eff_dist_km;   % eVTOL flies direct — anchors lower bound
ref_dist_max = base_dist * 1.25;

metrics_def = {
    'energy_per_km',    'lower',  0.3,       3.0,          'Energy';
    'peak_power_kW',    'lower',  50,        800,          'Energy';
    'soc_at_landing',   'higher', 0.20,      1.0,          'Performance';
    'mission_time_s',   'lower',  300,       2400,         'Performance';
    'peak_LD',          'higher', 5,         25,           'Aero';
    'V_best_range',     'lower',  20,        80,           'Aero';
    'eff_dist_km',      'lower',  base_dist, ref_dist_max, 'Operational';
    'energy_total_kWh', 'lower',  5,         200,          'Operational';
};

n = size(metrics_def, 1);
raw_e  = zeros(n, 1);
raw_s  = zeros(n, 1);
norm_e = zeros(n, 1);
norm_s = zeros(n, 1);
w_vec  = zeros(n, 1);

% Extract raw values, normalize, and collect weights
for i = 1:n
    field   = metrics_def{i, 1};
    dir     = metrics_def{i, 2};
    ref_min = metrics_def{i, 3};
    ref_max = metrics_def{i, 4};

    val_e    = m_e.(field);
    val_s    = m_s.(field);
    raw_e(i) = val_e;
    raw_s(i) = val_s;
    w_vec(i) = metric_weights.(field);

    % Normalize against reference range, clamp to [0,1]
    if strcmp(dir, 'lower')
        norm_e(i) = 1 - (val_e - ref_min) / (ref_max - ref_min);
        norm_s(i) = 1 - (val_s - ref_min) / (ref_max - ref_min);
    else
        norm_e(i) = (val_e - ref_min) / (ref_max - ref_min);
        norm_s(i) = (val_s - ref_min) / (ref_max - ref_min);
    end

    norm_e(i) = max(0, min(1, norm_e(i)));
    norm_s(i) = max(0, min(1, norm_s(i)));
end

% Final scores — direct weighted sum, no category averaging
score_e = sum(w_vec .* norm_e);
score_s = sum(w_vec .* norm_s);

if ~m_e.feasible; score_e = 0; end
if ~m_s.feasible; score_s = 0; end

% Category scores for display only -> computed as weighted average of constituent metrics
% Weights re-normalized within each category so they sum to 1 per category
categories = {'Energy', 'Performance', 'Aero', 'Operational'};
cat_score_e = struct();
cat_score_s = struct();
cat_fields  = {'energy', 'performance', 'aero', 'operational'};

for c = 1:length(categories)
    cat_label = categories{c};
    idx = strcmp(metrics_def(:, 5), cat_label);

    if sum(idx) == 0
        cat_score_e.(cat_fields{c}) = 0.5;
        cat_score_s.(cat_fields{c}) = 0.5;
        continue
    end

    % Re-normalize weights within category so display scores are in [0,1]
    w_cat   = w_vec(idx);
    w_cat_n = w_cat / sum(w_cat);

    cat_score_e.(cat_fields{c}) = sum(w_cat_n .* norm_e(idx));
    cat_score_s.(cat_fields{c}) = sum(w_cat_n .* norm_s(idx));
end

% Output
scores.evtol_total    = score_e;
scores.estol_total    = score_s;
scores.margin         = abs(score_e - score_s);

if score_e > score_s
    scores.winner = 'eVTOL';
elseif score_s > score_e
    scores.winner = 'eSTOL';
else
    scores.winner = 'Tie';
end

scores.evtol_category  = cat_score_e;
scores.estol_category  = cat_score_s;
scores.metric_weights  = metric_weights;
scores.metrics_def     = metrics_def;
scores.w_vec           = w_vec;
scores.raw_e           = raw_e;
scores.raw_s           = raw_s;
scores.norm_e          = norm_e;
scores.norm_s          = norm_s;

end
