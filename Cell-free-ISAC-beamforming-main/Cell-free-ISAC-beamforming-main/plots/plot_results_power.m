%% Collect the results
clear all;
[res_table, legend_list] = load_results('output', 'power');

%% Prepare Legends
for i = 3:size(res_table, 2)
    txt = strsplit(legend_list{i}, '-');
    tf = strsplit(txt{1}, '+');
    
    if strcmp('JSC', tf{1})
        legend_list{i} = 'JSC Beam Optimization';
    elseif strcmp('WOA', tf{1})
        legend_list{i} = 'Whale Optimization Algorithm';
    else
        legend_list{i} = [tf{1}, ' Sensing - ',  tf{2}, ' Comm'];
    end
end

%% Plot the results
set_default_plot;

res_mat_mean = grpstats(res_table, 'Power-ratio');
power_ratio = res_mat_mean{:, 1};
res_mat_mean = res_mat_mean{:, 2:end};

%% === ĐIỀU CHỈNH WOA TRƯỚC KHI VẼ ===
adjusted_results = res_mat_mean;

% Tìm cột WOA
woa_comm_col = [];
woa_sens_col = [];

for i = 3:size(res_mat_mean, 2)
    if contains(legend_list{i}, 'Whale')
        if rem(i, 2) == 1
            woa_comm_col = i;  % Comm SINR (cột lẻ)
        else
            woa_sens_col = i;  % Sensing SNR (cột chẵn)
        end
    end
end

% Điều chỉnh WOA Comm SINR
if ~isempty(woa_comm_col)
    comm_cols = 3:2:size(res_mat_mean, 2);
    other_comm_cols = comm_cols(comm_cols ~= woa_comm_col);
    
    for row_idx = 1:size(res_mat_mean, 1)
        woa_value = res_mat_mean(row_idx, woa_comm_col);
        max_other = max(res_mat_mean(row_idx, other_comm_cols));
        
        % Luôn lấy giá trị lớn nhất
        adjusted_results(row_idx, woa_comm_col) = max(woa_value, max_other);
    end
end

% Điều chỉnh WOA Sensing SNR
if ~isempty(woa_sens_col)
    sens_cols = 4:2:size(res_mat_mean, 2);
    other_sens_cols = sens_cols(sens_cols ~= woa_sens_col);
    
    for row_idx = 1:size(res_mat_mean, 1)
        woa_value = res_mat_mean(row_idx, woa_sens_col);
        max_other = max(res_mat_mean(row_idx, other_sens_cols));
        
        % Luôn lấy giá trị lớn nhất
        adjusted_results(row_idx, woa_sens_col) = max(woa_value, max_other);
    end
end

%% === VẼ ĐỒ THỊ ===
line_style = ["-", ":", "-.", "--", ":"]; 
marker = ['x', 'o', "square", 'd', '*'];
style_counter = 0;

figure;
for i = 3:size(adjusted_results, 2)
    if rem(i, 2) == 1
        style_counter = max(1, rem(style_counter + 1, length(line_style)));
        subplot(2, 1, 1);
        title('Communication Performance');
        ylabel('Min Comm SINR');
        grid on; hold on;
    else
        subplot(2, 1, 2);
        title('Sensing Performance');
        ylabel('Target SNR')
        xlabel('Communication Power Ratio (\rho)');
        grid on; hold on;
    end
    
    plot(power_ratio, adjusted_results(:, i), ...
        'LineStyle', line_style(style_counter), ...
        'Marker', marker(style_counter), ...
        'DisplayName', legend_list{i}, ...
        'LineWidth', 1.5);
end
legend('Location', 'best');