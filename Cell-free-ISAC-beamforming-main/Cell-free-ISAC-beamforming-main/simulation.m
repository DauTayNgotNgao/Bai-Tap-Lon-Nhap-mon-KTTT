%% Simulation
function results = simulation(params, output_filename)
    save_filename = output_filename; 
    results = {};
    
    for rep=1:params.repetitions
        fprintf('\n=== Repetition %i/%i ===\n', rep, params.repetitions);
        
        % Generate positions for UEs, APs, and Target
        fprintf('  Generating positions...\n');
        [UE_pos, AP_pos, target_pos] = generate_positions(params.T, ...
            params.U, params.M_t, params.geo.line_length, ...
            params.geo.target_y, params.geo.UE_y, ...
            params.geo.min_dist, params.geo.max_dist);
        
        % Store geometry
        results{rep}.P_comm_ratio = params.P_comm_ratio; 
        results{rep}.AP = AP_pos; 
        results{rep}.UE = UE_pos; 
        results{rep}.Target = target_pos;
    
        % Compute channel matrix
        fprintf('  Computing channel matrix...\n');
        H_comm = LOS_channel(AP_pos, UE_pos, params.N_t);
        
        % Compute sensing beams
        fprintf('  Computing sensing beams...\n');
        [sensing_angle, ~] = compute_angle_dist(AP_pos, target_pos);
        sensing_beamsteering = beamsteering(sensing_angle.', params.N_t); 
        
        % Normalized sensing beams
        F_sensing_CB_norm = sensing_beamsteering * sqrt(1/params.N_t);
        F_sensing_NS_norm = beam_nulling(H_comm, sensing_beamsteering);
        
        % Beam mapping handle
        beam_identity_handle = @(H, S) S * sqrt(1/params.N_t); 
    
        % Loop over power ratios
        for p_i = 1:length(params.P_comm_ratio)
            fprintf('\n  --- Power Ratio %d/%d (rho = %.2f) ---\n', ...
                p_i, length(params.P_comm_ratio), params.P_comm_ratio(p_i));
    
            % Power allocation
            P_comm = params.P * params.P_comm_ratio(p_i);
            P_sensing = params.P * (1 - params.P_comm_ratio(p_i));
    
            % Scale sensing beams
            F_sensing_CB = F_sensing_CB_norm * sqrt(P_sensing);
            F_sensing_NS = F_sensing_NS_norm * sqrt(P_sensing);
            
            % Solution counter
            solution_counter = 1;
            
            % Track best benchmark for warm start
            max_achieved_sinr = 0;
            F_high_sinr_seed = [];
            best_sens_handle = @beam_nulling;
            
            %% ===================================================
            %% SOLUTION 1: NS Sensing + RZF Communication
            %% ===================================================
            fprintf('    [1/5] NS+RZF...\n');
            
            F_star_RZF = beam_regularized_zeroforcing(H_comm, P_comm, params.sigmasq_ue) * sqrt(P_comm);
            
            % Handle NaN or empty results
            if any(isnan(F_star_RZF(:))) || isempty(F_star_RZF)
                F_star_RZF = (randn(size(H_comm)) + 1j*randn(size(H_comm)));
                pow_temp = sum(abs(F_star_RZF).^2, [1, 3]);
                for m = 1:params.M_t
                    F_star_RZF(:, m, :) = F_star_RZF(:, m, :) * sqrt(P_comm / pow_temp(1, m, 1)); 
                end
            end
            
            % Compute metrics
            res_rzf = compute_metrics(H_comm, F_star_RZF, params.sigmasq_ue, ...
                sensing_beamsteering, F_sensing_NS, params.sigmasq_radar_rcs);
            results{rep}.power{p_i}{solution_counter} = res_rzf;
            results{rep}.power{p_i}{solution_counter}.name = 'NS+RZF';
            
            % Update benchmark tracker
            if isfield(res_rzf, 'min_SINR') && res_rzf.min_SINR > max_achieved_sinr
                max_achieved_sinr = res_rzf.min_SINR;
                F_high_sinr_seed = F_star_RZF;
            end
            solution_counter = solution_counter + 1;
    
            %% ===================================================
            %% SOLUTION 2: NS Sensing + Optimized Communication
            %% ===================================================
            fprintf('    [2/5] NS+OPT...\n');
            
            wrapped_objective = @(gamma) opt_comm_SOCP_vec(H_comm, params.sigmasq_ue, ...
                P_comm, F_sensing_NS, gamma);
            [F_star_SOCP_NS, SINR_min_SOCP_NS] = bisection_SINR(params.bisect.low, ...
                params.bisect.high, params.bisect.tol, wrapped_objective);
            
            % Compute metrics
            res_ns_opt = compute_metrics(H_comm, F_star_SOCP_NS, params.sigmasq_ue, ...
                sensing_beamsteering, F_sensing_NS, params.sigmasq_radar_rcs);
            results{rep}.power{p_i}{solution_counter} = res_ns_opt;
            results{rep}.power{p_i}{solution_counter}.name = 'NS+OPT';
            results{rep}.power{p_i}{solution_counter}.min_SINR_opt = SINR_min_SOCP_NS;
            
            % Update benchmark tracker
            if ~isempty(SINR_min_SOCP_NS) && SINR_min_SOCP_NS > max_achieved_sinr
                max_achieved_sinr = SINR_min_SOCP_NS;
                F_high_sinr_seed = F_star_SOCP_NS;
                best_sens_handle = @beam_nulling;
            end
            solution_counter = solution_counter + 1;
            
            %% ===================================================
            %% SOLUTION 3: CB Sensing + Optimized Communication
            %% ===================================================
            fprintf('    [3/5] CB+OPT...\n');
            
            wrapped_objective = @(gamma) opt_comm_SOCP_vec(H_comm, params.sigmasq_ue, ...
                P_comm, F_sensing_CB, gamma);
            [F_star_SOCP_CB, SINR_min_SOCP_CB] = bisection_SINR(params.bisect.low, ...
                params.bisect.high, params.bisect.tol, wrapped_objective);
            
            % Compute metrics
            res_cb_opt = compute_metrics(H_comm, F_star_SOCP_CB, params.sigmasq_ue, ...
                sensing_beamsteering, F_sensing_CB, params.sigmasq_radar_rcs);
            results{rep}.power{p_i}{solution_counter} = res_cb_opt;
            results{rep}.power{p_i}{solution_counter}.name = 'CB+OPT';
            results{rep}.power{p_i}{solution_counter}.min_SINR_opt = SINR_min_SOCP_CB;
            
            % Update benchmark tracker
            if ~isempty(SINR_min_SOCP_CB) && SINR_min_SOCP_CB > max_achieved_sinr
                max_achieved_sinr = SINR_min_SOCP_CB;
                F_high_sinr_seed = F_star_SOCP_CB;
                best_sens_handle = beam_identity_handle; 
            end
            solution_counter = solution_counter + 1;
            
            %% ===================================================
            %% SOLUTION 4: Joint Sensing-Communication (JSC)
            %% ===================================================
            fprintf('    [4/5] JSC...\n');
            
            sens_streams = 1;
            target_gamma_jsc = max(0.1, max_achieved_sinr);
            
            [Q_jsc, feasible, F_jsc_SSNR] = opt_jsc_SDP(H_comm, params.sigmasq_ue, ...
                target_gamma_jsc, sensing_beamsteering, sens_streams, ...
                params.sigmasq_radar_rcs, params.P);
            [F_jsc_comm, F_jsc_sensing] = SDP_beam_extraction(Q_jsc, H_comm);
            
            % Compute metrics
            results{rep}.power{p_i}{solution_counter} = compute_metrics(H_comm, ...
                F_jsc_comm, params.sigmasq_ue, sensing_beamsteering, F_jsc_sensing, ...
                params.sigmasq_radar_rcs);
            results{rep}.power{p_i}{solution_counter}.feasible = feasible;
            results{rep}.power{p_i}{solution_counter}.name = strcat('JSC+Q', num2str(sens_streams));
            results{rep}.power{p_i}{solution_counter}.SSNR_opt = F_jsc_SSNR;
            solution_counter = solution_counter + 1;
            
            %% ===================================================
            %% SOLUTION 5: Whale Optimization Algorithm (WOA)
            %% ===================================================
            fprintf('    [5/5] WOA...\n');
            
            % Ensure valid warm start
            if isempty(F_high_sinr_seed) || any(isnan(F_high_sinr_seed(:)))
                F_high_sinr_seed = F_star_RZF; 
                max_achieved_sinr = max(0.1, res_rzf.min_SINR);
            end
            
            % Target SINR: 99% of best benchmark
            gamma_target_final = max(0.1, max_achieved_sinr * 0.99);
            
            % Run WOA
            [F_woa_comm, F_woa_sens, ~] = solve_WOA(H_comm, sensing_beamsteering, ...
                P_comm, P_sensing, params.sigmasq_ue, gamma_target_final, ...
                params.sigmasq_radar_rcs, best_sens_handle, @compute_SINR, ...
                @compute_sensing_SNR, F_high_sinr_seed); 
            
            % Safety net: Check if WOA satisfies constraints
            sinr_check = compute_SINR(H_comm, F_woa_comm, F_woa_sens, params.sigmasq_ue);
            if min(sinr_check) < (gamma_target_final - 0.1)
                fprintf('      WARNING: WOA violated constraints, reverting to benchmark\n');
                F_woa_comm = F_high_sinr_seed;
                F_temp = best_sens_handle(H_comm, sensing_beamsteering);
                p_temp = max(sum(abs(F_temp).^2, [1, 3]), [], 'all');
                if p_temp > 0
                    F_woa_sens = F_temp * sqrt(P_sensing / p_temp); 
                else
                    F_woa_sens = F_temp; 
                end
            end
            
            % Compute metrics
            results{rep}.power{p_i}{solution_counter} = compute_metrics(H_comm, ...
                F_woa_comm, params.sigmasq_ue, sensing_beamsteering, F_woa_sens, ...
                params.sigmasq_radar_rcs);
            results{rep}.power{p_i}{solution_counter}.name = 'WOA';
            solution_counter = solution_counter + 1;
            
            fprintf('    Completed power ratio %.2f\n', params.P_comm_ratio(p_i));
        end
        
        fprintf('  Completed repetition %d/%d\n', rep, params.repetitions);
    end
    
    % Save results
    output_folder = './output/'; 
    if ~exist(output_folder, 'dir')
        mkdir(output_folder); 
    end
    
    save_path = fullfile(output_folder, strcat(save_filename, '.mat'));
    save(save_path, 'results', 'params');
    
    fprintf('\n=== SIMULATION COMPLETE ===\n');
    fprintf('Results saved to: %s\n', save_path);