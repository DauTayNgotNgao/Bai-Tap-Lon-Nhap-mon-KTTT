function [F_comm, F_sens, best_fit] = solve_WOA(H_comm, sensing_beamsteering, P_comm, P_sensing, sigmasq_comm, gamma_target, sigmasq_radar_rcs, beam_mapping_handle, compute_SINR_handle, compute_sensing_SNR_handle, F_warm_start)
% SOLVE_WOA: Whale Optimization Algorithm using Deb's Feasibility Rules
% Chiến thuật: Ưu tiên thỏa mãn Comm SINR trước, sau đó mới tối ưu Sensing SNR.

    %% 1. Parameters
    max_iter = 200;      % Số vòng lặp tối đa
    pop_size = 50;       % Kích thước quần thể
    
    [U, M, N] = size(H_comm);
    dim = 2 * U * M * N; % Số chiều (real + imag)
    
    %% 2. Initialization (Bias Mode)
    x_warm = [real(F_warm_start(:)); imag(F_warm_start(:))]';
    population = zeros(pop_size, dim);
    
    % --- 1. THE LEADER: Warm Start ---
    population(1, :) = x_warm;
    
    % --- 2. THE FOLLOWERS: 20% biến thể nhẹ ---
    num_followers = round(pop_size * 0.2);
    for i = 2:(num_followers + 1)
        population(i, :) = x_warm + 0.05 * randn(1, dim); 
    end
    
    % --- 3. THE EXPLORERS: Random ---
    for i = (num_followers + 2):pop_size
        population(i, :) = -1 + 2*rand(1, dim); 
    end
    
    % Lưu trữ Objective (Sensing) và Violation (Comm Error) riêng biệt
    pop_obj = zeros(pop_size, 1);
    pop_vio = zeros(pop_size, 1);
    
    % Đánh giá ban đầu
    for i = 1:pop_size
        [pop_obj(i), pop_vio(i)] = evaluate_candidate(population(i, :), H_comm, sensing_beamsteering, P_comm, P_sensing, sigmasq_comm, gamma_target, sigmasq_radar_rcs, beam_mapping_handle, compute_SINR_handle, compute_sensing_SNR_handle, U, M, N);
    end
    
    % Tìm Best Solution ban đầu theo Deb's Rules
    best_idx = 1;
    for i = 2:pop_size
        if check_deb_wins(pop_obj(i), pop_vio(i), pop_obj(best_idx), pop_vio(best_idx))
            best_idx = i;
        end
    end
    best_sol = population(best_idx, :);
    best_fit = pop_obj(best_idx);
    
    %% 3. WOA Main Loop
    a = 2; % Tham số giảm tuyến tính từ 2 về 0
    
    for iter = 1:max_iter
        a = 2 - iter * (2 / max_iter); % Giảm tuyến tính
        
        for i = 1:pop_size
            r1 = rand();
            r2 = rand();
            
            A = 2 * a * r1 - a;
            C = 2 * r2;
            
            p = rand();
            b = 1;  % Hằng số xoắn ốc
            l = (rand() - 0.5) * 2; % Random trong [-1, 1]
            
            if p < 0.5
                if abs(A) < 1
                    % --- Encircling prey (Shrinking) ---
                    D = abs(C * best_sol - population(i, :));
                    new_position = best_sol - A * D;
                else
                    % --- Exploration (Search for prey) ---
                    rand_idx = randi(pop_size);
                    X_rand = population(rand_idx, :);
                    D = abs(C * X_rand - population(i, :));
                    new_position = X_rand - A * D;
                end
            else
                % --- Spiral updating position ---
                D_prime = abs(best_sol - population(i, :));
                new_position = D_prime .* exp(b * l) .* cos(2 * pi * l) + best_sol;
            end
            
            % Giới hạn biên
            new_position = max(-1, min(1, new_position));
            
            % Đánh giá vị trí mới
            [new_obj, new_vio] = evaluate_candidate(new_position, H_comm, sensing_beamsteering, P_comm, P_sensing, sigmasq_comm, gamma_target, sigmasq_radar_rcs, beam_mapping_handle, compute_SINR_handle, compute_sensing_SNR_handle, U, M, N);
            
            % So sánh theo Deb's Rules
            if check_deb_wins(new_obj, new_vio, pop_obj(i), pop_vio(i))
                population(i, :) = new_position;
                pop_obj(i) = new_obj;
                pop_vio(i) = new_vio;
                
                % Update Global Best
                if check_deb_wins(new_obj, new_vio, best_fit, pop_vio(best_idx))
                    best_sol = new_position;
                    best_fit = new_obj;
                    best_idx = i;
                end
            end
        end
    end
    
    % Reconstruct Final Result
    [F_comm, F_sens] = reconstruct_matrices(best_sol, H_comm, sensing_beamsteering, P_comm, P_sensing, U, M, N, beam_mapping_handle);
end

%% --- HELPER FUNCTIONS ---

function [obj, vio] = evaluate_candidate(x, H_comm, sensing_beamsteering, P_comm, P_sensing, sigmasq_comm, gamma_target, sigmasq_radar_rcs, beam_mapping_handle, compute_SINR_handle, compute_sensing_SNR_handle, U, M, N)
    [F_comm, F_sens] = reconstruct_matrices(x, H_comm, sensing_beamsteering, P_comm, P_sensing, U, M, N, beam_mapping_handle);
    
    % 1. Tính Objective (Sensing SNR)
    obj = compute_sensing_SNR_handle(sigmasq_radar_rcs, sensing_beamsteering, F_comm, F_sens);
    
    % 2. Tính Violation (Comm SINR)
    SINR = compute_SINR_handle(H_comm, F_comm, F_sens, sigmasq_comm);
    min_sinr = min(SINR);
    
    target_threshold = gamma_target - 1e-4;
    
    if min_sinr >= target_threshold
        vio = 0;
    else
        vio = target_threshold - min_sinr;
    end
end

function wins = check_deb_wins(obj_A, vio_A, obj_B, vio_B)
    if (vio_A == 0) && (vio_B > 0)
        wins = true;
    elseif (vio_A > 0) && (vio_B > 0)
        wins = (vio_A < vio_B);
    elseif (vio_A == 0) && (vio_B == 0)
        wins = (obj_A > obj_B);
    else
        wins = false;
    end
end

function [F_comm, F_sens] = reconstruct_matrices(x, H_comm, sensing_beamsteering, P_comm, P_sensing, U, M, N, beam_mapping_handle)
    num_elements = U * M * N;
    F_real = x(1:num_elements);
    F_imag = x(num_elements+1:end);
    F_comm = reshape(F_real + 1j * F_imag, [U, M, N]);
    
    % Strict Power Scaling (Comm)
    pow_per_ap_comm = sum(abs(F_comm).^2, [1, 3]); 
    for m = 1:M
        if pow_per_ap_comm(1, m, 1) > P_comm + 1e-4
            F_comm(:, m, :) = F_comm(:, m, :) * sqrt(P_comm / pow_per_ap_comm(1, m, 1));
        end
    end
    
    % Strict Power Scaling (Sensing)
    F_sens_base = beam_mapping_handle(H_comm, sensing_beamsteering); 
    curr_p = max(sum(abs(F_sens_base).^2, [1, 3]), [], 'all');
    if curr_p > 0
        F_sens = F_sens_base * sqrt(P_sensing / curr_p);
    else
        F_sens = F_sens_base;
    end
end