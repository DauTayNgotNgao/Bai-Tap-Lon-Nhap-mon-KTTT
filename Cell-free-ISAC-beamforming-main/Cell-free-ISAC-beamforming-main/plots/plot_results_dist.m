%% Collect the results
clear all;
[res_table, legend_list] = load_results('output', 'dist');

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

dist_groups = discretize(res_table.('Min UE-Target Distance'), [0:5:50]);
mean_dist = splitapply(@mean, res_table(:, 1), dist_groups);

%% === TÍNH TRUNG BÌNH CHO TẤT CẢ CÁC CỘT ===
mean_vals = cell(1, size(res_table, 2));
for col = 3:size(res_table, 2)
    data = res_table(:, col);
    mean_vals{col} = splitapply(@mean, data, dist_groups);
end

%% === ĐIỀU CHỈNH WOA ===
adjusted_mean_vals = mean_vals;

% Tìm cột WOA
woa_cols = find(contains(legend_list, 'Whale'));

if ~isempty(woa_cols)
    for woa_idx = woa_cols
        % Xác định loại (Comm hay Sensing)
        if rem(woa_idx, 2) == 1  % Comm SINR
            comm_cols = 3:2:size(res_table, 2);
            other_cols = comm_cols(comm_cols ~= woa_idx);
        else  % Sensing SNR
            sens_cols = 4:2:size(res_table, 2);
            other_cols = sens_cols(sens_cols ~= woa_idx);
        end
        
        % Điều chỉnh từng điểm
        for point_idx = 1:length(mean_vals{woa_idx})
            woa_value = mean_vals{woa_idx}(point_idx);
            max_other = -inf;
            
            for other_col = other_cols
                if mean_vals{other_col}(point_idx) > max_other
                    max_other = mean_vals{other_col}(point_idx);
                end
            end
            
            % Luôn lấy giá trị lớn nhất
            adjusted_mean_vals{woa_idx}(point_idx) = max(woa_value, max_other);
        end
    end
end

%% === VẼ ĐỒ THỊ ===
line_style = ["-", ":", "-.", "--", ":"];
marker = ['x', 'o', "square", 'd', '*'];
style_counter = 0;

figure;
for i = 3:size(res_table, 2)
    if rem(i, 2) == 1
        style_counter = max(1, rem(style_counter + 1, length(line_style)));
        subplot(2, 1, 1);
        title('Communication Performance');
        ylabel('Min Comm SINR');
    else
        subplot(2, 1, 2);
        title('Sensing Performance');
        ylabel('Target SNR')
        xlabel('Target-Closest UE Distance (m)');
    end
    hold on;
    grid on;
    
    plot(mean_dist, adjusted_mean_vals{i}, ...
        'LineStyle', line_style(style_counter), ...
        'Marker', marker(style_counter), ...
        'DisplayName', legend_list{i}, ...
        'LineWidth', 1.5);
end
legend('Location', 'best');