function experiment_data_collection()
% EXPERIMENT_DATA_COLLECTION Main script for gathering experimental data
% Performs impulse, step, and multisine experiments on thermal model

    clc; close all;
    
    %% ========== EXPERIMENTAL PARAMETERS ==========
    params = struct();
    params.dt = 1;              % Sampling time [s]
    params.t_final = 600;       % Experiment duration [s]
    params.T0 = 23;             % Initial temperature [°C]
    params.Ta = 23;             % Ambient temperature [°C]
    params.model_type = 'nonlinear';  % 'linear' or 'nonlinear'
    
    % Input signal parameters
    step_amplitude = 50;        % Step input amplitude [%]
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
    
    Q_step = @(t) step_amplitude * (t >= 0);
    [T_step, t_step, Q_step_data] = thermal_model(Q_step, params);
    
    % Save data
    step_data = struct('t', t_step, 'T', T_step, 'Q', Q_step_data, 'params', params);
    save('step_response_data.mat', 'step_data');
    fprintf('  Saved: step_response_data.mat\n');
    
    %% ========== EXPERIMENT 2: IMPULSE RESPONSE ==========
    fprintf('Running Impulse Response Experiment...\n');
    
    Q_impulse = @(t) impulse_amplitude * (t >= 0 & t < impulse_duration);
    [T_impulse, t_impulse, Q_impulse_data] = thermal_model(Q_impulse, params);
    
    % Save data
    impulse_data = struct('t', t_impulse, 'T', T_impulse, 'Q', Q_impulse_data, 'params', params);
    save('impulse_response_data.mat', 'impulse_data');
    fprintf('  Saved: impulse_response_data.mat\n');
    
    %% ========== EXPERIMENT 3: MULTISINE RESPONSE ==========
    fprintf('Running Multisine Response Experiment...\n');
    
    % Generate multisine frequencies (log-spaced)
    freqs = logspace(log10(freq_min), log10(freq_max), n_freq);
    
    % Random phases for better excitation
    phases = 2*pi*rand(1, n_freq);
    
    % Multisine signal
    Q_multisine = @(t) multisine_offset + multisine_amplitude * ...
        sum(sin(2*pi*freqs'.*t + phases'), 1) / n_freq;
    
    [T_multisine, t_multisine, Q_multisine_data] = thermal_model(Q_multisine, params);
    
    % Save data
    multisine_data = struct('t', t_multisine, 'T', T_multisine, 'Q', Q_multisine_data, ...
                            'params', params, 'freqs', freqs, 'phases', phases);
    save('multisine_response_data.mat', 'multisine_data');
    fprintf('  Saved: multisine_response_data.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', [100, 100, 1200, 800]);
    
    % Step response
    subplot(3,2,1);
    plot(t_step, Q_step_data, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Step Response - Input');
    grid on;
    
    subplot(3,2,2);
    plot(t_step, T_step, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Step Response - Output');
    grid on;
    
    % Impulse response
    subplot(3,2,3);
    plot(t_impulse, Q_impulse_data, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Impulse Response - Input');
    grid on;
    
    subplot(3,2,4);
    plot(t_impulse, T_impulse, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Impulse Response - Output');
    grid on;
    
    % Multisine response
    subplot(3,2,5);
    plot(t_multisine, Q_multisine_data, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Input [%]');
    title('Multisine Response - Input');
    grid on;
    
    subplot(3,2,6);
    plot(t_multisine, T_multisine, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Multisine Response - Output');
    grid on;
    
    sgtitle('Data Collection Experiments');
    saveas(gcf, 'data_collection_plots.png');
    fprintf('  Saved: data_collection_plots.png\n');
    
    fprintf('\nData collection complete!\n');
end