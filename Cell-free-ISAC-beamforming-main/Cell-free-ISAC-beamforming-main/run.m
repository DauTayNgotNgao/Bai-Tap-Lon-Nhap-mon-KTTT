cvx_path = 'C:\Users\Admin\Documents\MATLAB\cvx-w64\cvx';
addpath(genpath(cvx_path));
addpath(genpath('./'));

%% ========================================================================
%% SIMULATION FOR FIG. 3 - POWER RATIO
%% ========================================================================
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║     SIMULATION FOR FIG. 3 (POWER RATIO)                   ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');

clear all;
eval('sim_params'); % Load parameters
output_file = 'power_data';

tic;
simulation(params, output_file);
time_power = toc;

fprintf('\n✓ Power simulation completed in %.1f seconds\n', time_power);

%% ========================================================================
%% SIMULATION FOR FIG. 4 - DISTANCE
%% ========================================================================
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║     SIMULATION FOR FIG. 4 (DISTANCE)                      ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');

clear all;
eval('sim_params'); % Load parameters
params.P_comm_ratio = [0.5]; % Fixed power ratio for distance analysis

tic;
% Loop through distance ranges
distance_ranges = 0:5:45;
total_ranges = length(distance_ranges);

for idx = 1:total_ranges
    min_dist = distance_ranges(idx);
    
    fprintf('\n[%d/%d] Distance Range: %d-%d meters\n', ...
        idx, total_ranges, min_dist, min_dist+5);
    
    params.geo.min_dist = min_dist;
    params.geo.max_dist = min_dist + 5;
    
    output_file = strcat('dist_data', num2str(min_dist));
    simulation(params, output_file);
end

time_dist = toc;
fprintf('\n✓ Distance simulation completed in %.1f seconds\n', time_dist);

%% ========================================================================
%% PLOT BOTH FIGURES
%% ========================================================================
fprintf('\n');
fprintf('    GENERATING PLOTS                                      \n');

% Plot Figure 3 - Power Ratio
fprintf('\n→ Plotting Figure 3 (Power Ratio)...\n');
plot_results_power;
set(gcf, 'Name', 'Figure 3: Power Ratio Analysis', 'NumberTitle', 'off');
drawnow;

% Plot Figure 4 - Distance
fprintf('→ Plotting Figure 4 (Distance)...\n');
plot_results_dist;
set(gcf, 'Name', 'Figure 4: Distance Analysis', 'NumberTitle', 'off');
drawnow;


%% SUMMARY
fprintf('\n');
fprintf('  Power Simulation:    %.1f seconds                        \n', time_power);
fprintf('  Distance Simulation: %.1f seconds                        \n', time_dist);
fprintf('  Total Time:          %.1f seconds (%.1f minutes)         \n', ...
    time_power + time_dist, (time_power + time_dist)/60);
fprintf('\n');
fprintf('  Figure 3: Power Ratio Analysis - DISPLAYED               \n');
fprintf(' Figure 4: Distance Analysis - DISPLAYED                  \n');
fprintf(' Results saved in: ./output/                              \n');
fprintf('\n');
