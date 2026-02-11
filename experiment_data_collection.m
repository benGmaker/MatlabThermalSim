function experiment_data_collection()
% EXPERIMENT_DATA_COLLECTION Main script for gathering experimental data
% Performs impulse, step, and multisine experiments on thermal model
% IMPROVED: Better step response data for DMC controller

    clc; close all;
    
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    %% ========== EXPERIMENTAL PARAMETERS ==========
    params = struct();
    params.dt = 1;              % Sampling time [s]
    params.t_final = 600;       % Experiment duration [s]
    params.T0 = 23;             % Initial temperature [°C]
    params.Ta = 23;             % Ambient temperature [°C]
    params.model_type = 'nonlinear';  % 'linear' or 'nonlinear'
    
    % Input signal parameters
    step_amplitude = 50;        % Step input amplitude [%]
    step_delay = 50;            % Delay before step [s] - NEW: allows baseline measurement
    impulse_amplitude = 100;    % Impulse amplitude [%]
    impulse_duration = 10;      % Impulse duration [s]
    
    % Multisine parameters
    multisine_amplitude = 30;   % Multisine amplitude [%]
    multisine_offset = 20;      % Multisine DC offset [%]
    freq_min = 0.001;           % Min frequency [Hz]
    freq_max = 0.05;            % Max frequency [Hz]
    n_freq = 10;                % Number of frequencies
    
    %% ========== EXPERIMENT 1: STEP RESPONSE ==========
    fprintf('Running Step Response Experiment...\n');
    
    % Step with initial steady-state period (IMPROVED for DMC)
    Q_step = @(t) step_amplitude * (t >= step_delay);
    [T_step, t_step, Q_step_data] = thermal_model(Q_step, params);
    
    % Calculate additional metrics for verification
    step_idx = find(Q_step_data > 0, 1, 'first');
    T_initial = mean(T_step(1:max(1,step_idx-1)));
    T_final = mean(T_step(end-50:end));  % Average last 50 samples
    steady_state_gain = (T_final - T_initial) / step_amplitude;
    
    fprintf('  Initial temperature: %.2f °C\n', T_initial);
    fprintf('  Final temperature: %.2f °C\n', T_final);
    fprintf('  Steady-state gain: %.4f °C/%%\n', steady_state_gain);
    fprintf('  Step occurs at t = %.1f s (sample %d)\n', step_delay, step_idx);
    
    % Save data with additional metadata
    step_data = struct('t', t_step, 'T', T_step, 'Q', Q_step_data, ...
                       'params', params, ...
                       'step_amplitude', step_amplitude, ...
                       'step_delay', step_delay, ...
                       'step_idx', step_idx, ...
                       'steady_state_gain', steady_state_gain);
    save('results/step_response_data.mat', 'step_data');
    fprintf('  Saved: results/step_response_data.mat\n');
    
    %% ========== EXPERIMENT 2: IMPULSE RESPONSE ==========
    fprintf('Running Impulse Response Experiment...\n');
    
    impulse_delay = 50;  % Delay before impulse
    Q_impulse = @(t) impulse_amplitude * (t >= impulse_delay & t < impulse_delay + impulse_duration);
    [T_impulse, t_impulse, Q_impulse_data] = thermal_model(Q_impulse, params);
    
    % Save data with metadata
    impulse_data = struct('t', t_impulse, 'T', T_impulse, 'Q', Q_impulse_data, ...
                          'params', params, ...
                          'impulse_amplitude', impulse_amplitude, ...
                          'impulse_duration', impulse_duration, ...
                          'impulse_delay', impulse_delay);
    save('results/impulse_response_data.mat', 'impulse_data');
    fprintf('  Saved: results/impulse_response_data.mat\n');
    
    %% ========== EXPERIMENT 3: MULTISINE RESPONSE ==========
    fprintf('Running Multisine Response Experiment...\n');
    
    % Generate multisine frequencies (log-spaced)
    freqs = logspace(log10(freq_min), log10(freq_max), n_freq);
    
    % Random phases for better excitation
    rng(42);  % Set seed for reproducibility
    phases = 2*pi*rand(1, n_freq);
    
    % Multisine signal
    Q_multisine = @(t) multisine_offset + multisine_amplitude * ...
        sum(sin(2*pi*freqs'.*t + phases'), 1) / n_freq;
    
    [T_multisine, t_multisine, Q_multisine_data] = thermal_model(Q_multisine, params);
    
    % Save data
    multisine_data = struct('t', t_multisine, 'T', T_multisine, 'Q', Q_multisine_data, ...
                            'params', params, 'freqs', freqs, 'phases', phases, ...
                            'amplitude', multisine_amplitude, 'offset', multisine_offset);
    save('results/multisine_response_data.mat', 'multisine_data');
    fprintf('  Saved: results/multisine_response_data.mat\n');
    
    %% ========== EXPERIMENT 4: DOUBLET TEST (OPTIONAL - GOOD FOR DMC) ==========
    fprintf('Running Doublet Test Experiment...\n');
    
    doublet_amplitude = 40;     % Doublet amplitude [%]
    doublet_duration = 100;     % Duration of each pulse [s]
    doublet_delay = 50;         % Initial delay [s]
    
    % Doublet: positive pulse followed by negative pulse
    Q_doublet = @(t) doublet_amplitude * (t >= doublet_delay & t < doublet_delay + doublet_duration) - ...
                     doublet_amplitude * (t >= doublet_delay + doublet_duration & t < doublet_delay + 2*doublet_duration);
    
    [T_doublet, t_doublet, Q_doublet_data] = thermal_model(Q_doublet, params);
    
    % Save data
    doublet_data = struct('t', t_doublet, 'T', T_doublet, 'Q', Q_doublet_data, ...
                          'params', params, ...
                          'doublet_amplitude', doublet_amplitude, ...
                          'doublet_duration', doublet_duration, ...
                          'doublet_delay', doublet_delay);
    save('results/doublet_response_data.mat', 'doublet_data');
    fprintf('  Saved: results/doublet_response_data.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', [100, 100, 1600, 900]);
    
    % Step response
    subplot(4,2,1);
    plot(t_step, Q_step_data, 'b', 'LineWidth', 1.5);
    xline(step_delay, 'r--', 'Step', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Step Response - Input');
    grid on;
    
    subplot(4,2,2);
    plot(t_step, T_step, 'r', 'LineWidth', 1.5); hold on;
    yline(T_initial, 'k--', 'Initial');
    yline(T_final, 'g--', 'Final');
    xline(step_delay, 'b--', 'Step');
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title(sprintf('Step Response - Output (Gain: %.4f)', steady_state_gain));
    grid on;
    legend('Response', 'Initial', 'Final', 'Step Time', 'Location', 'best');
    
    % Impulse response
    subplot(4,2,3);
    plot(t_impulse, Q_impulse_data, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Impulse Response - Input');
    grid on;
    
    subplot(4,2,4);
    plot(t_impulse, T_impulse, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Impulse Response - Output');
    grid on;
    
    % Multisine response
    subplot(4,2,5);
    plot(t_multisine, Q_multisine_data, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Multisine Response - Input');
    grid on;
    
    subplot(4,2,6);
    plot(t_multisine, T_multisine, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Multisine Response - Output');
    grid on;
    
    % Doublet response
    subplot(4,2,7);
    plot(t_doublet, Q_doublet_data, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Doublet Response - Input');
    grid on;
    
    subplot(4,2,8);
    plot(t_doublet, T_doublet, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Doublet Response - Output');
    grid on;
    
    sgtitle('Data Collection Experiments');
    saveas(gcf, 'results/data_collection_plots.png');
    fprintf('  Saved: results/data_collection_plots.png\n');
    
    %% ========== STEP RESPONSE ANALYSIS (FOR DMC VALIDATION) ==========
    fprintf('\nStep Response Analysis for DMC:\n');
    
    % Normalize step response to unit step
    y_response = T_step(step_idx:end) - T_initial;
    u_step_magnitude = step_amplitude;
    y_unit_step = y_response / u_step_magnitude;
    
    % Display first 20 step response coefficients
    fprintf('  First 20 step response coefficients (unit step):\n');
    n_display = min(20, length(y_unit_step));
    for i = 1:n_display
        fprintf('    S(%2d) = %.6f\n', i, y_unit_step(i));
    end
    fprintf('  ...\n');
    fprintf('  S(end) = %.6f (steady-state)\n', y_unit_step(end));
    
    % Check for monotonicity (thermal systems should be monotonic)
    if all(diff(y_unit_step(1:min(100,end))) >= -1e-6)
        fprintf('  ✓ Step response is monotonic (good for DMC)\n');
    else
        fprintf('  ⚠ Step response is non-monotonic (check system)\n');
    end
    
    % Time constant estimate (63.2% of final value)
    final_value = y_unit_step(end);
    tau_idx = find(y_unit_step >= 0.632 * final_value, 1, 'first');
    if ~isempty(tau_idx)
        tau = tau_idx * params.dt;
        fprintf('  Time constant (τ): %.1f s\n', tau);
        fprintf('  Recommended N for DMC: %d samples (≈4τ)\n', ceil(4*tau/params.dt));
    end
    
    fprintf('\nData collection complete!\n');
end