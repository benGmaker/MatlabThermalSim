function [y_noisy, noise_stats] = add_measurement_noise(y_clean, config)
% ADD_MEASUREMENT_NOISE Add measurement noise to signal based on desired SNR
%
% Inputs:
%   y_clean - Clean signal (column vector)
%   config  - Configuration struct with noise parameters
%
% Outputs:
%   y_noisy - Noisy signal
%   noise_stats - Struct with noise statistics including achieved SNR
%
% The function accurately generates noise to achieve the desired SNR in dB

    if ~config.noise.enable
        % Noise disabled
        y_noisy = y_clean;
        noise_stats = struct();
        noise_stats.noise_enabled = false;
        noise_stats.SNR_desired_dB = NaN;
        noise_stats.SNR_achieved_dB = NaN;
        return;
    end
    
    % Set random seed for reproducibility
    rng(config.noise.seed);
    
    %% Calculate signal power
    % Remove DC component for SNR calculation
    signal_mean = mean(y_clean);
    signal_ac = y_clean - signal_mean;
    signal_power = mean(signal_ac.^2);
    
    if signal_power < eps
        warning('Signal has zero or near-zero power. Cannot add meaningful noise.');
        y_noisy = y_clean;
        noise_stats.noise_enabled = false;
        return;
    end
    
    %% Calculate required noise power from desired SNR
    % SNR_dB = 10 * log10(P_signal / P_noise)
    % P_noise = P_signal / 10^(SNR_dB/10)
    SNR_linear = 10^(config.noise.SNR_dB / 10);
    noise_power_target = signal_power / SNR_linear;
    
    %% Generate noise based on type
    n = length(y_clean);
    
    switch lower(config.noise.type)
        case 'gaussian'
            % White Gaussian noise
            noise = randn(n, 1);
            
        case 'uniform'
            % Uniform noise
            noise = rand(n, 1) - 0.5;  % Zero mean
            
        case 'colored'
            % Colored (filtered) noise
            % Create a low-pass filter to color the noise
            if config.noise.colored_cutoff_freq <= 0 || config.noise.colored_cutoff_freq >= 1
                warning('Invalid cutoff frequency. Using 0.1');
                cutoff = 0.1;
            else
                cutoff = config.noise.colored_cutoff_freq;
            end
            
            % Generate white noise first
            white_noise = randn(n, 1);
            
            % Design Butterworth filter
            [b, a] = butter(config.noise.colored_filter_order, cutoff);
            
            % Apply filter to create colored noise
            noise = filtfilt(b, a, white_noise);
            
        otherwise
            error('Unknown noise type: %s. Use ''gaussian'', ''uniform'', or ''colored''.', config.noise.type);
    end
    
    %% Scale noise to achieve target power
    % Current noise power
    noise = noise - mean(noise);  % Ensure zero mean
    current_noise_power = mean(noise.^2);
    
    % Scale noise
    scale_factor = sqrt(noise_power_target / current_noise_power);
    noise_scaled = noise * scale_factor;
    
    %% Add noise to signal
    y_noisy = y_clean + noise_scaled;
    
    %% Calculate achieved SNR
    % SNR based on AC component
    noise_power_achieved = mean((noise_scaled).^2);
    SNR_achieved_linear = signal_power / noise_power_achieved;
    SNR_achieved_dB = 10 * log10(SNR_achieved_linear);
    
    %% Store statistics
    noise_stats = struct();
    noise_stats.noise_enabled = true;
    noise_stats.noise_type = config.noise.type;
    noise_stats.SNR_desired_dB = config.noise.SNR_dB;
    noise_stats.SNR_achieved_dB = SNR_achieved_dB;
    noise_stats.signal_power = signal_power;
    noise_stats.noise_power = noise_power_achieved;
    noise_stats.signal_mean = signal_mean;
    noise_stats.signal_std = std(signal_ac);
    noise_stats.noise_std = std(noise_scaled);
    noise_stats.noise_max = max(abs(noise_scaled));
    noise_stats.random_seed = config.noise.seed;
    
    % Calculate RMSE
    noise_stats.RMSE = sqrt(mean((y_noisy - y_clean).^2));
end