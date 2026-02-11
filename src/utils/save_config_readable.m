
function save_config_readable(config, filename)
% SAVE_CONFIG_READABLE Save configuration in human-readable text format
%
% Inputs:
%   config - Configuration struct
%   filename - Output filename (e.g., 'config_used.txt')

    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not open file for writing: %s', filename);
    end
    
    % Write header
    fprintf(fid, '================================================================================\n');
    fprintf(fid, '  THERMAL CONTROL SYSTEM - CONFIGURATION FILE\n');
    fprintf(fid, '================================================================================\n');
    fprintf(fid, 'Generated: %s\n', char(config.metadata.created));
    fprintf(fid, 'MATLAB Version: %s\n', config.metadata.matlab_version);
    fprintf(fid, '================================================================================\n\n');
    
    % Simulation Parameters
    fprintf(fid, '--- SIMULATION PARAMETERS ---\n');
    fprintf(fid, 'Simulation Time:              %d s\n', config.simulation.t_sim);
    fprintf(fid, 'Sampling Time:                %.1f s\n', config.simulation.dt);
    fprintf(fid, '\n');
    
    % Setpoint Profile
    fprintf(fid, '--- SETPOINT PROFILE ---\n');
    fprintf(fid, 'Times:                        %s s\n', mat2str(config.setpoint.times));
    fprintf(fid, 'Values:                       %s °C\n', mat2str(config.setpoint.values));
    fprintf(fid, '\n');
    
    % Control Constraints
    fprintf(fid, '--- CONTROL CONSTRAINTS (SHARED) ---\n');
    fprintf(fid, 'Minimum Input (u_min):        %d %%\n', config.constraints.u_min);
    fprintf(fid, 'Maximum Input (u_max):        %d %%\n', config.constraints.u_max);
    fprintf(fid, 'Max Rate of Change (du_max):  %d %%/sample\n', config.constraints.du_max);
    fprintf(fid, '\n');
    
    % MPC Parameters
    fprintf(fid, '--- MPC PARAMETERS ---\n');
    fprintf(fid, 'Prediction Horizon (P):       %d samples\n', config.MPC.P);
    fprintf(fid, 'Control Horizon (M):          %d samples\n', config.MPC.M);
    fprintf(fid, 'Output Tracking Weight (Q):   %.3f\n', config.MPC.Q_weight);
    fprintf(fid, 'Input Change Penalty (R):     %.3f\n', config.MPC.R_weight);
    fprintf(fid, '\n');
    
    % DMC Parameters
    fprintf(fid, '--- DMC PARAMETERS ---\n');
    fprintf(fid, 'Prediction Horizon (P):       %d samples\n', config.DMC.P);
    fprintf(fid, 'Control Horizon (M):          %d samples\n', config.DMC.M);
    fprintf(fid, 'Model Horizon (N):            %d samples\n', config.DMC.N);
    fprintf(fid, 'Output Tracking Weight (Q):   %.3f\n', config.DMC.Q_weight);
    fprintf(fid, 'Input Change Penalty (R):     %.3f\n', config.DMC.R_weight);
    fprintf(fid, 'Step Response Source:         %s\n', config.DMC.step_response_source);
    fprintf(fid, '\n');
    
    % DeePC Parameters
    fprintf(fid, '--- DeePC PARAMETERS ---\n');
    fprintf(fid, 'Past Horizon (T_ini):         %d samples\n', config.DeePC.T_ini);
    fprintf(fid, 'Future Horizon (N):           %d samples\n', config.DeePC.N);
    fprintf(fid, 'Slack Penalty Output (λ_y):   %.1f\n', config.DeePC.lambda_y);
    fprintf(fid, 'Slack Penalty Hankel (λ_g):   %.1f\n', config.DeePC.lambda_g);
    fprintf(fid, 'Input Regularization (λ_u):   %.1f\n', config.DeePC.lambda_u);
    fprintf(fid, '\n');
    
    % Data Collection Parameters
    fprintf(fid, '--- DATA COLLECTION PARAMETERS ---\n');
    fprintf(fid, 'Experiment Duration:          %d s\n', config.data_collection.t_final);
    fprintf(fid, 'Sampling Time:                %.1f s\n', config.data_collection.dt);
    fprintf(fid, 'Step Amplitude:               %d %%\n', config.data_collection.step_amplitude);
    fprintf(fid, 'Step Delay:                   %d s\n', config.data_collection.step_delay);
    fprintf(fid, 'Impulse Amplitude:            %d %%\n', config.data_collection.impulse_amplitude);
    fprintf(fid, 'Impulse Duration:             %d s\n', config.data_collection.impulse_duration);
    fprintf(fid, 'Impulse Delay:                %d s\n', config.data_collection.impulse_delay);
    fprintf(fid, 'Multisine Amplitude:          %d %%\n', config.data_collection.multisine_amplitude);
    fprintf(fid, 'Multisine Offset:             %d %%\n', config.data_collection.multisine_offset);
    fprintf(fid, 'Frequency Range:              %.4f - %.4f Hz\n', ...
            config.data_collection.freq_min, config.data_collection.freq_max);
    fprintf(fid, 'Number of Frequencies:        %d\n', config.data_collection.n_freq);
    fprintf(fid, 'Doublet Amplitude:            %d %%\n', config.data_collection.doublet_amplitude);
    fprintf(fid, 'Doublet Duration:             %d s\n', config.data_collection.doublet_duration);
    fprintf(fid, 'Doublet Delay:                %d s\n', config.data_collection.doublet_delay);
    fprintf(fid, '\n');
    
    % System Identification Parameters
    fprintf(fid, '--- SYSTEM IDENTIFICATION PARAMETERS ---\n');
    fprintf(fid, 'Dataset Choice:               %s\n', config.system_id.dataset_choice);
    fprintf(fid, 'Number of Poles:              %d\n', config.system_id.n_poles);
    fprintf(fid, 'Number of Zeros:              %d\n', config.system_id.n_zeros);
    fprintf(fid, 'Validation Fraction:          %.1f%%\n', config.system_id.validation_fraction * 100);
    fprintf(fid, '\n');
    
    % Thermal Model Parameters
    fprintf(fid, '--- THERMAL MODEL PARAMETERS ---\n');
    fprintf(fid, 'Initial Temperature (T0):     %.1f °C\n', config.thermal_model.T0);
    fprintf(fid, 'Ambient Temperature (Ta):     %.1f °C\n', config.thermal_model.Ta);
    fprintf(fid, 'Heat Transfer Coeff (U):      %.1f W/m²-K\n', config.thermal_model.U);
    fprintf(fid, 'Surface Area (A):             %.2e m²\n', config.thermal_model.A);
    fprintf(fid, 'Mass (m):                     %.2e kg\n', config.thermal_model.m);
    fprintf(fid, 'Heat Capacity (Cp):           %.1f J/kg-K\n', config.thermal_model.Cp);
    fprintf(fid, 'Heater Efficiency (α):        %.3f\n', config.thermal_model.alpha);
    fprintf(fid, 'Radiation Coeff (ε):          %.2f\n', config.thermal_model.eps);
    fprintf(fid, 'Stefan-Boltzmann (σ):         %.2e W/m²-K⁴\n', config.thermal_model.sigma);
    fprintf(fid, 'Model Type:                   %s\n', config.thermal_model.model_type);
    fprintf(fid, '\n');
    
    % Plotting Parameters
    fprintf(fid, '--- PLOTTING PARAMETERS ---\n');
    fprintf(fid, 'MPC Color (RGB):              [%.1f, %.1f, %.1f]\n', ...
            config.plotting.colors.MPC(1), config.plotting.colors.MPC(2), config.plotting.colors.MPC(3));
    fprintf(fid, 'DMC Color (RGB):              [%.1f, %.1f, %.1f]\n', ...
            config.plotting.colors.DMC(1), config.plotting.colors.DMC(2), config.plotting.colors.DMC(3));
    fprintf(fid, 'DeePC Color (RGB):            [%.1f, %.1f, %.1f]\n', ...
            config.plotting.colors.DeePC(1), config.plotting.colors.DeePC(2), config.plotting.colors.DeePC(3));
    fprintf(fid, 'Line Width:                   %.1f\n', config.plotting.linewidth);
    fprintf(fid, 'Figure Size:                  [%d, %d, %d, %d]\n', ...
            config.plotting.figure_size(1), config.plotting.figure_size(2), ...
            config.plotting.figure_size(3), config.plotting.figure_size(4));
    fprintf(fid, '\n');
    
    % Footer
    fprintf(fid, '================================================================================\n');
    fprintf(fid, 'END OF CONFIGURATION\n');
    fprintf(fid, '================================================================================\n');
    
    fclose(fid);
end