function system_identification(config)
% SYSTEM_IDENTIFICATION Identifies transfer function models from experimental data
% IMPROVED: Uses centralized configuration

    if nargin < 1
        config = config_simulation();
    end
    
    close all;
    
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    fprintf('========================================\n');
    fprintf('  System Identification\n');
    fprintf('========================================\n\n');
    
    %% ========== PARAMETERS FROM CONFIG ==========
    dataset_choice = config.system_id.dataset_choice;
    n_poles = config.system_id.n_poles;
    n_zeros = config.system_id.n_zeros;
    validation_fraction = config.system_id.validation_fraction;
    
    fprintf('Identification Parameters:\n');
    fprintf('  Dataset: %s\n', dataset_choice);
    fprintf('  Model order: %d poles, %d zeros\n', n_poles, n_zeros);
    fprintf('  Validation fraction: %.1f%%\n', validation_fraction * 100);
    fprintf('\n');
    
    %% ========== LOAD DATA ==========
    fprintf('Loading %s response data...\n', dataset_choice);
    
    switch dataset_choice
        case 'step'
            data = load('results/step_response_data.mat');
            exp_data = data.step_data;
        case 'impulse'
            data = load('results/impulse_response_data.mat');
            exp_data = data.impulse_data;
        case 'multisine'
            data = load('results/multisine_response_data.mat');
            exp_data = data.multisine_data;
        otherwise
            error('Unknown dataset choice: %s', dataset_choice);
    end
    
    %% ========== PREPARE DATA ==========
    t = exp_data.t;
    u = exp_data.Q;
    y = exp_data.T;
    dt = exp_data.params.dt;
    
    % Remove mean
    u_mean = mean(u);
    y_mean = mean(y);
    u_centered = u - u_mean;
    y_centered = y - y_mean;
    
    fprintf('  Input mean: %.2f, std: %.2f\n', u_mean, std(u));
    fprintf('  Output mean: %.2f, std: %.2f\n', y_mean, std(y));
    
    % Create iddata object
    data_full = iddata(y_centered, u_centered, dt);
    
    % Split into estimation and validation
    n_samples = length(t);
    n_val = round(validation_fraction * n_samples);
    n_est = n_samples - n_val;
    
    data_est = data_full(1:n_est);
    data_val = data_full(n_est+1:end);
    
    fprintf('  Estimation samples: %d\n', n_est);
    fprintf('  Validation samples: %d\n\n', n_val);
    
    %% ========== TRANSFER FUNCTION ESTIMATION ==========
    fprintf('Estimating transfer function model...\n');
    
    sys_tf = tfest(data_est, n_poles, n_zeros);
    
    fprintf('  Identified Transfer Function:\n');
    disp(sys_tf);
    
    % Get step response characteristics
    stepinfo_tf = stepinfo(sys_tf);
    fprintf('  Rise time: %.2f s\n', stepinfo_tf.RiseTime);
    fprintf('  Settling time: %.2f s\n', stepinfo_tf.SettlingTime);
    
    %% ========== STATE-SPACE ESTIMATION ==========
    fprintf('\nEstimating state-space model...\n');
    
    sys_ss = ssest(data_est, n_poles);
    
    fprintf('  State-space model order: %d\n', order(sys_ss));
    
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
    
    save('results/identified_models.mat', 'identified_models');
    fprintf('\nSaved: results/identified_models.mat\n');
    
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