function experiment_data_collection(config)
% EXPERIMENT_DATA_COLLECTION Main script for gathering experimental data
% Performs impulse, step, and multisine experiments on thermal model
% IMPROVED: Uses centralized configuration and adds measurement noise

    if nargin < 1
        config = config_simulation();
    end
    
    close all;
    
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    fprintf('========================================\n');
    fprintf('  Data Collection Experiments\n');
    fprintf('========================================\n\n');
    
    %% ========== SETUP PARAMETERS FROM CONFIG ==========
    params = struct();
    params.dt = config.data_collection.dt;
    params.t_final = config.data_collection.t_final;
    params.T0 = config.thermal_model.T0;
    params.Ta = config.thermal_model.Ta;
    params.model_type = config.thermal_model.model_type;
    
    fprintf('Experiment Parameters:\n');
    fprintf('  Duration: %d s\n', params.t_final);
    fprintf('  Sampling time: %.1f s\n', params.dt);
    fprintf('  Model type: %s\n', params.model_type);
    
    % Display noise settings
    if config.noise.enable
        fprintf('  Noise: ENABLED\n');
        fprintf('    Type: %s\n', config.noise.type);
        fprintf('    Target SNR: %.1f dB\n', config.noise.SNR_dB);
        if strcmpi(config.noise.type, 'colored')
            fprintf('    Filter order: %d\n', config.noise.colored_filter_order);
            fprintf('    Cutoff freq: %.3f (normalized)\n', config.noise.colored_cutoff_freq);
        end
    else
        fprintf('  Noise: DISABLED\n');
    end
    fprintf('\n');
    
    %% ========== EXPERIMENT 1: STEP RESPONSE ==========
    fprintf('Running Step Response Experiment...\n');
    
    step_amplitude = config.data_collection.step_amplitude;
    step_delay = config.data_collection.step_delay;
    
    Q_step = @(t) step_amplitude * (t >= step_delay);
    [T_step_clean, t_step, Q_step_data] = thermal_model(Q_step, params);
    
    % Add noise
    [T_step, noise_stats_step] = add_measurement_noise(T_step_clean, config);
    
    % Calculate metrics
    step_idx = find(Q_step_data > 0, 1, 'first');
    T_initial = mean(T_step(1:max(1,step_idx-1)));
    T_final = mean(T_step(end-50:end));
    steady_state_gain = (T_final - T_initial) / step_amplitude;
    
    fprintf('  Initial temperature: %.2f °C\n', T_initial);
    fprintf('  Final temperature: %.2f °C\n', T_final);
    fprintf('  Steady-state gain: %.4f °C/%%\n', steady_state_gain);
    fprintf('  Step occurs at t = %.1f s (sample %d)\n', step_delay, step_idx);
    
    if config.noise.enable
        fprintf('  Noise Statistics:\n');
        fprintf('    Desired SNR: %.2f dB\n', noise_stats_step.SNR_desired_dB);
        fprintf('    Achieved SNR: %.2f dB\n', noise_stats_step.SNR_achieved_dB);
        fprintf('    Noise RMSE: %.4f °C\n', noise_stats_step.RMSE);
        fprintf('    Noise Std: %.4f °C\n', noise_stats_step.noise_std);
    end
    
    % Save data
    step_data = struct('t', t_step, 'T', T_step, 'T_clean', T_step_clean, 'Q', Q_step_data, ...
                       'params', params, ...
                       'step_amplitude', step_amplitude, ...
                       'step_delay', step_delay, ...
                       'step_idx', step_idx, ...
                       'steady_state_gain', steady_state_gain, ...
                       'noise_stats', noise_stats_step, ...
                       'config', config);
    save('results/step_response_data.mat', 'step_data');
    fprintf('  Saved: results/step_response_data.mat\n\n');
    
    %% ========== EXPERIMENT 2: IMPULSE RESPONSE ==========
    fprintf('Running Impulse Response Experiment...\n');
    
    impulse_amplitude = config.data_collection.impulse_amplitude;
    impulse_duration = config.data_collection.impulse_duration;
    impulse_delay = config.data_collection.impulse_delay;
    
    Q_impulse = @(t) impulse_amplitude * (t >= impulse_delay & t < impulse_delay + impulse_duration);
    [T_impulse_clean, t_impulse, Q_impulse_data] = thermal_model(Q_impulse, params);
    
    % Add noise
    [T_impulse, noise_stats_impulse] = add_measurement_noise(T_impulse_clean, config);
    
    if config.noise.enable
        fprintf('  Noise Statistics:\n');
        fprintf('    Achieved SNR: %.2f dB\n', noise_stats_impulse.SNR_achieved_dB);
        fprintf('    Noise RMSE: %.4f °C\n', noise_stats_impulse.RMSE);
    end
    
    % Save data
    impulse_data = struct('t', t_impulse, 'T', T_impulse, 'T_clean', T_impulse_clean, 'Q', Q_impulse_data, ...
                          'params', params, ...
                          'impulse_amplitude', impulse_amplitude, ...
                          'impulse_duration', impulse_duration, ...
                          'impulse_delay', impulse_delay, ...
                          'noise_stats', noise_stats_impulse, ...
                          'config', config);
    save('results/impulse_response_data.mat', 'impulse_data');
    fprintf('  Saved: results/impulse_response_data.mat\n\n');
    
    %% ========== EXPERIMENT 3: MULTISINE RESPONSE ==========
    fprintf('Running Multisine Response Experiment...\n');
    
    multisine_amplitude = config.data_collection.multisine_amplitude;
    multisine_offset = config.data_collection.multisine_offset;
    freq_min = config.data_collection.freq_min;
    freq_max = config.data_collection.freq_max;
    n_freq = config.data_collection.n_freq;
    
    % Generate multisine frequencies
    freqs = logspace(log10(freq_min), log10(freq_max), n_freq);
    
    rng(42);
    phases = 2*pi*rand(1, n_freq);
    
    Q_multisine = @(t) multisine_offset + multisine_amplitude * ...
        sum(sin(2*pi*freqs'.*t + phases'), 1) / n_freq;
    
    [T_multisine_clean, t_multisine, Q_multisine_data] = thermal_model(Q_multisine, params);
    
    % Add noise
    [T_multisine, noise_stats_multisine] = add_measurement_noise(T_multisine_clean, config);
    
    fprintf('  Frequencies: %.4f to %.4f Hz\n', freq_min, freq_max);
    fprintf('  Number of frequencies: %d\n', n_freq);
    
    if config.noise.enable
        fprintf('  Noise Statistics:\n');
        fprintf('    Achieved SNR: %.2f dB\n', noise_stats_multisine.SNR_achieved_dB);
        fprintf('    Noise RMSE: %.4f °C\n', noise_stats_multisine.RMSE);
    end
    
    % Save data
    multisine_data = struct('t', t_multisine, 'T', T_multisine, 'T_clean', T_multisine_clean, ...
                            'Q', Q_multisine_data, ...
                            'params', params, 'freqs', freqs, 'phases', phases, ...
                            'amplitude', multisine_amplitude, 'offset', multisine_offset, ...
                            'noise_stats', noise_stats_multisine, ...
                            'config', config);
    save('results/multisine_response_data.mat', 'multisine_data');
    fprintf('  Saved: results/multisine_response_data.mat\n\n');
    
    %% ========== EXPERIMENT 4: DOUBLET TEST ==========
    fprintf('Running Doublet Test Experiment...\n');
    
    doublet_amplitude = config.data_collection.doublet_amplitude;
    doublet_duration = config.data_collection.doublet_duration;
    doublet_delay = config.data_collection.doublet_delay;
    
    Q_doublet = @(t) doublet_amplitude * (t >= doublet_delay & t < doublet_delay + doublet_duration) - ...
                     doublet_amplitude * (t >= doublet_delay + doublet_duration & t < doublet_delay + 2*doublet_duration);
    
    [T_doublet_clean, t_doublet, Q_doublet_data] = thermal_model(Q_doublet, params);
    
    % Add noise
    [T_doublet, noise_stats_doublet] = add_measurement_noise(T_doublet_clean, config);
    
    if config.noise.enable
        fprintf('  Noise Statistics:\n');
        fprintf('    Achieved SNR: %.2f dB\n', noise_stats_doublet.SNR_achieved_dB);
        fprintf('    Noise RMSE: %.4f °C\n', noise_stats_doublet.RMSE);
    end
    
    % Save data
    doublet_data = struct('t', t_doublet, 'T', T_doublet, 'T_clean', T_doublet_clean, 'Q', Q_doublet_data, ...
                          'params', params, ...
                          'doublet_amplitude', doublet_amplitude, ...
                          'doublet_duration', doublet_duration, ...
                          'doublet_delay', doublet_delay, ...
                          'noise_stats', noise_stats_doublet, ...
                          'config', config);
    save('results/doublet_response_data.mat', 'doublet_data');
    fprintf('  Saved: results/doublet_response_data.mat\n\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', config.plotting.figure_size);
    
    % Step response
    subplot(4,3,1);
    plot(t_step, Q_step_data, 'b', 'LineWidth', config.plotting.linewidth);
    xline(step_delay, 'r--', 'Step', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Step Response - Input');
    grid on;
    
    subplot(4,3,2);
    plot(t_step, T_step_clean, 'k--', 'LineWidth', 1, 'DisplayName', 'Clean'); hold on;
    plot(t_step, T_step, 'r', 'LineWidth', config.plotting.linewidth, 'DisplayName', 'Noisy');
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title(sprintf('Step Response - Output (SNR: %.1f dB)', noise_stats_step.SNR_achieved_dB));
    legend('Location', 'best');
    grid on;
    
    subplot(4,3,3);
    plot(t_step, T_step - T_step_clean, 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Noise [°C]');
    title(sprintf('Step - Noise (σ=%.3f)', noise_stats_step.noise_std));
    grid on;
    
    % Impulse response
    subplot(4,3,4);
    plot(t_impulse, Q_impulse_data, 'b', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Impulse Response - Input');
    grid on;
    
    subplot(4,3,5);
    plot(t_impulse, T_impulse_clean, 'k--', 'LineWidth', 1); hold on;
    plot(t_impulse, T_impulse, 'r', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title(sprintf('Impulse Response - Output (SNR: %.1f dB)', noise_stats_impulse.SNR_achieved_dB));
    grid on;
    
    subplot(4,3,6);
    plot(t_impulse, T_impulse - T_impulse_clean, 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Noise [°C]');
    title(sprintf('Impulse - Noise (σ=%.3f)', noise_stats_impulse.noise_std));
    grid on;
    
    % Multisine response
    subplot(4,3,7);
    plot(t_multisine, Q_multisine_data, 'b', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Multisine Response - Input');
    grid on;
    
    subplot(4,3,8);
    plot(t_multisine, T_multisine_clean, 'k--', 'LineWidth', 1); hold on;
    plot(t_multisine, T_multisine, 'r', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title(sprintf('Multisine Response - Output (SNR: %.1f dB)', noise_stats_multisine.SNR_achieved_dB));
    grid on;
    
    subplot(4,3,9);
    plot(t_multisine, T_multisine - T_multisine_clean, 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Noise [°C]');
    title(sprintf('Multisine - Noise (σ=%.3f)', noise_stats_multisine.noise_std));
    grid on;
    
    % Doublet response
    subplot(4,3,10);
    plot(t_doublet, Q_doublet_data, 'b', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Doublet Response - Input');
    grid on;
    
    subplot(4,3,11);
    plot(t_doublet, T_doublet_clean, 'k--', 'LineWidth', 1); hold on;
    plot(t_doublet, T_doublet, 'r', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title(sprintf('Doublet Response - Output (SNR: %.1f dB)', noise_stats_doublet.SNR_achieved_dB));
    grid on;
    
    subplot(4,3,12);
    plot(t_doublet, T_doublet - T_doublet_clean, 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Noise [°C]');
    title(sprintf('Doublet - Noise (σ=%.3f)', noise_stats_doublet.noise_std));
    grid on;
    
    if config.noise.enable
        sgtitle(sprintf('Data Collection Experiments (Noise: %s, Target SNR: %.1f dB)', ...
                        config.noise.type, config.noise.SNR_dB));
    else
        sgtitle('Data Collection Experiments (No Noise)');
    end
    saveas(gcf, 'results/data_collection_plots.png');
    fprintf('  Saved: results/data_collection_plots.png\n');
    
    %% ========== STEP RESPONSE ANALYSIS ==========
    fprintf('\nStep Response Analysis for DMC:\n');
    
    % Use noisy data for analysis
    y_response = T_step(step_idx:end) - T_initial;
    u_step_magnitude = step_amplitude;
    y_unit_step = y_response / u_step_magnitude;
    
    % Display first 20 coefficients
    fprintf('  First 20 step response coefficients (unit step):\n');
    n_display = min(20, length(y_unit_step));
    for i = 1:n_display
        fprintf('    S(%2d) = %.6f\n', i, y_unit_step(i));
    end
    fprintf('  ...\n');
    fprintf('  S(end) = %.6f (steady-state)\n', y_unit_step(end));
    
    % Check monotonicity
    if all(diff(y_unit_step(1:min(100,end))) >= -1e-6)
        fprintf('  ✓ Step response is monotonic (good for DMC)\n');
    else
        fprintf('  ⚠ Step response is non-monotonic (check system or noise level)\n');
    end
    
    % Time constant
    final_value = y_unit_step(end);
    tau_idx = find(y_unit_step >= 0.632 * final_value, 1, 'first');
    if ~isempty(tau_idx)
        tau = tau_idx * params.dt;
        fprintf('  Time constant (τ): %.1f s\n', tau);
        fprintf('  Recommended N for DMC: %d samples (≈4τ)\n', ceil(4*tau/params.dt));
    end
    
    fprintf('\nData collection complete!\n');
end