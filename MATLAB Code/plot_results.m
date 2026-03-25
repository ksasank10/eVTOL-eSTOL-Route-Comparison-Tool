function plot_results(m_e, m_s, scores, route)
% PLOT_RESULTS  Generates the four comparison figures for the App.
%
%   Inputs:
%     m_e    — eVTOL metrics from mission_sim.m
%     m_s    — eSTOL metrics from mission_sim.m
%     scores — output from scoring_framework.m
%     route  — route struct

figure('Name','UAM Comparison Results','Position',[60 60 1300 900]);

%  Plot 1: Power vs time
subplot(2,2,1); hold on;
plot(m_e.t_full/60, m_e.P_full/1e3, 'b', 'LineWidth',1.5, 'DisplayName','eVTOL');
plot(m_s.t_full/60, m_s.P_full/1e3, 'r', 'LineWidth',1.5, 'DisplayName','eSTOL');
xlabel('Time [min]'); ylabel('Power [kW]');
title('Power vs time — full mission'); legend; grid on;

%  Plot 2: Battery SoC vs distance
subplot(2,2,2); hold on;
soc_e = 1 - cumtrapz(m_e.t_full, m_e.P_full) / (m_e.energy_total_kWh / ...
    (1 - m_e.soc_at_landing) * 3.6e6);
soc_s = 1 - cumtrapz(m_s.t_full, m_s.P_full) / (m_s.energy_total_kWh / ...
    (1 - m_s.soc_at_landing) * 3.6e6);
plot(m_e.x_full/1e3, soc_e*100, 'b', 'LineWidth',1.5, 'DisplayName','eVTOL');
plot(m_s.x_full/1e3, soc_s*100, 'r', 'LineWidth',1.5, 'DisplayName','eSTOL');
yline(route.battery_reserve*100, 'k--', 'Reserve', 'LineWidth',0.8);
xlabel('Ground range [km]'); ylabel('Battery SoC [%]');
title('Battery state of charge vs distance'); legend; grid on;

%  Plot 3: Scoring bar chart
subplot(2,2,3);
categories  = {'Energy','Performance','Aero','Operational'};
cat_fields  = {'energy','performance','aero','operational'};
scores_e = zeros(1,4); scores_s = zeros(1,4);
for i = 1:4
    scores_e(i) = scores.evtol_category.(cat_fields{i});
    scores_s(i) = scores.estol_category.(cat_fields{i});
end
x = 1:4;
bar(x - 0.2, scores_e, 0.35, 'FaceColor', [0.22 0.48 0.75], 'DisplayName','eVTOL');
hold on;
bar(x + 0.2, scores_s, 0.35, 'FaceColor', [0.80 0.33 0.17], 'DisplayName','eSTOL');
set(gca, 'XTick', x, 'XTickLabel', categories);
ylabel('Weighted score [0-1]'); ylim([0 1.1]);
title('Category scores (weighted avg within category)'); legend; grid on;

% Add total score annotation
text(0.98, 0.97, sprintf('eVTOL: %.3f', scores.evtol_total), ...
    'Units','normalized','HorizontalAlignment','right','FontSize',9,'Color',[0.22 0.48 0.75]);
text(0.98, 0.90, sprintf('eSTOL: %.3f', scores.estol_total), ...
    'Units','normalized','HorizontalAlignment','right','FontSize',9,'Color',[0.80 0.33 0.17]);
text(0.98, 0.83, sprintf('Winner: %s', scores.winner), ...
    'Units','normalized','HorizontalAlignment','right','FontSize',9,'FontWeight','bold');

%  Plot 4: Energy efficiency vs route distance
subplot(2,2,4);
% This is pre-computed in day3_verify — here we just show the mission point
C = constants();
distances = 10:5:100;
epk_e = zeros(size(distances));
epk_s = zeros(size(distances));
[ev_in, es_in, ~] = default_params();
evtol_ac = build_aircraft(ev_in);
estol_ac = build_aircraft(es_in);

for i = 1:length(distances)
    r = route; r.distance_m = distances(i)*1e3;
    me = mission_energy(evtol_ac, r, C);
    ms = mission_energy(estol_ac, r, C);
    epk_e(i) = me.energy_per_km;
    epk_s(i) = ms.energy_per_km;
end

hold on;
plot(distances, epk_e, 'b-o', 'LineWidth',1.5, 'MarkerSize',4, 'DisplayName','eVTOL');
plot(distances, epk_s, 'r-s', 'LineWidth',1.5, 'MarkerSize',4, 'DisplayName','eSTOL');
xline(route.distance_m/1e3, 'k--', 'This route', 'LineWidth',0.8, ...
    'HandleVisibility','off');
xlabel('Route distance [km]'); ylabel('Energy per km [kWh/km]');
title('Energy efficiency vs route distance'); legend; grid on;

sgtitle(sprintf('UAM Comparison — %.0f km route | Winner: %s (%.3f vs %.3f)', ...
    route.distance_m/1e3, scores.winner, ...
    scores.evtol_total, scores.estol_total), 'FontSize',12);

saveas(gcf, '../results/final_comparison.png');
fprintf('Figure saved: results/final_comparison.png\n');

end
