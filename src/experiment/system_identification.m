function system_identification(config)
% SYSTEM_IDENTIFICATION Identifies transfer function models from experimental data
% IMPROVED: Uses centralized configuration

    if nargin < 1
        config = config_simulation();
    end
    
    close all;
    
    % Create results directory if it doesn't exist
    if ~exist('results/data', 'dir')
        mkdir('results/data');
    end
    
    fprintf('========================================\n');
    fprintf('  System Identification\n');
    fprintf('========================================\n\n');
    
    %% ========== PARAMETERS FROM CONFIG ==========
    dataset_choice = config.dataset_choice;
    n_poles = config.system_id.n_poles;
    n_zeros = config.system_id.n_zeros;
    
    fprintf('Identification Parameters:\n');
    fprintf('  Dataset: %s\n', dataset_choice);
    fprintf('  Model order: %d poles, %d zeros\n', n_poles, n_zeros);
    fprintf('\n');
    
    %% ========== LOAD DATA ==========
    exp_data = load_predictive_data(config);
    val_cfg.dataset_choice = "multisine2";
    val_data = load_predictive_data(val_cfg);
    
    %% ========== PREPARE DATA ==========
    t = exp_data.t;
    u = exp_data.Q;
    y = exp_data.T;
    dt = exp_data.params.dt;
    
    % Remove mean optionally 
    if config.data.centering 
        u_mean = mean(u); y_mean = mean(y);
    else
        u_mean = 0; y_mean = 0;
    end 

    u_centered = u - u_mean;
    y_centered = y - y_mean;
    
    fprintf('  Input mean: %.2f, std: %.2f\n', u_mean, std(u));
    fprintf('  Output mean: %.2f, std: %.2f\n', y_mean, std(y));
    
    % Create iddata object
    data_full = iddata(y_centered, u_centered, dt);
    
    % Removing initial step from the data to fit 
    if config.dataset_choice=="step"
        idx_begin = config.data_collection.step_delay - 1;
    else
        idx_begin = 1;
    end

    data_est = data_full(idx_begin:end);
    data_val = iddata(val_data.T, val_data.Q, val_data.dt);
    
    %% ========== TRANSFER FUNCTION ESTIMATION ==========
    fprintf('Estimating transfer function model...\n');
    
    sys_tf = tfest(data_est, n_poles, n_zeros);
    
    fprintf('  Identified Transfer Function:\n');
    disp(sys_tf);
    
    % Get step response characteristics
    stepinfo_tf = stepinfo(sys_tf);
    fprintf('  Rise time: %.2f s\n', stepinfo_tf.RiseTime);
    fprintf('  Settling time: %.2f s\n', stepinfo_tf.SettlingTime);


    fprintf('\Converting state-space model...\n')
    sys_ss_c = ss(sys_tf);
 
    % Discrete-time state-space using the experiment sample time dt
    % (choose method: 'zoh' is typical for sampled-data inputs)
    sys_ss = c2d(sys_ss_c, dt, 'zoh');
    
    % Optional: make it explicit in output
    sys_ss.Ts = dt;
    
    fprintf('  Discretized SS sample time Ts = %.6g s\n', sys_ss.Ts);
        
    %% ========== MODEL VALIDATION ==========
    fprintf('\nValidating models...\n');
    
    [y_tf, fit_tf, ~] = compare(data_val, sys_tf);
    [y_ss, fit_ss, ~] = compare(data_val, sys_ss);
    
    fprintf('  Transfer Function fit: %.2f%%\n', fit_tf);
    fprintf('  State-Space fit: %.2f%%\n', fit_ss);
    
    %% ========== SAVE MODELS ==========
    identified_models = struct();
    identified_models.sys_tf = sys_tf;
    identified_models.sys_ss = sys_ss;
    identified_models.u_mean = u_mean;
    identified_models.y_mean = y_mean;
    identified_models.dt = dt;
    identified_models.fit_tf = fit_tf;
    identified_models.fit_ss = fit_ss;
    identified_models.dataset_used = dataset_choice;
    identified_models.config = config;
    
    save('results/data/identified_models.mat', 'identified_models');
    fprintf('\nSaved: results/data/identified_models.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', config.plotting.figure_size);
    
    % Estimation data fit
    subplot(2,3,1);
    compare(data_est, sys_tf, sys_ss);
    title('Model Fit - Estimation Data');
    legend('Measured', sprintf('TF (fit: %.1f%%)', fit_tf), ...
           sprintf('SS (fit: %.1f%%)', fit_ss));
    grid on;
    
    % Validation data fit
    subplot(2,3,2);
    compare(data_val, sys_tf, sys_ss);
    title('Model Fit - Validation Data');
    grid on;
    
    % Step response comparison
    subplot(2,3,3);
    step(sys_tf, sys_ss);
    title('Step Response Comparison');
    legend('Transfer Function', 'State-Space');
    grid on;
    
    % Bode plot - TF
    subplot(2,3,4);
    bode(sys_tf);
    title('Bode Plot - Transfer Function');
    grid on;
    
    % Pole-zero map - TF
    subplot(2,3,5);
    pzmap(sys_tf);
    title('Pole-Zero Map - Transfer Function');
    grid on;
    
    % Residuals
    subplot(2,3,6);
    residue_data = resid(data_val, sys_tf);
    plot(residue_data.y);
    xlabel('Sample'); ylabel('Residual [°C]');
    title('Model Residuals - Validation Data');
    grid on;
    
    sgtitle('System Identification Results');
    saveas(gcf, 'results/system_identification_plots.png');
    fprintf('  Saved: results/system_identification_plots.png\n');
    
    fprintf('\nSystem identification complete!\n');
end