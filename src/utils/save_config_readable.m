function save_config_readable(config, filename)
% SAVE_CONFIG_READABLE Save configuration in human-readable text format
%
% Inputs:
%   config   - Configuration struct (from config_simulation())
%   filename - Output filename (e.g., 'config_used.txt')

    fid = fopen(filename, 'w');
    if fid == -1
        error('Could not open file for writing: %s', filename);
    end

    % =============================== HEADER ===============================
    fprintf(fid, '================================================================================\n');
    fprintf(fid, '  THERMAL CONTROL SYSTEM - CONFIGURATION FILE\n');
    fprintf(fid, '================================================================================\n');
    if isfield(config,'metadata') && isfield(config.metadata,'created')
        fprintf(fid, 'Generated: %s\n', char(config.metadata.created));
    else
        fprintf(fid, 'Generated: (unknown)\n');
    end
    if isfield(config,'metadata') && isfield(config.metadata,'matlab_version')
        fprintf(fid, 'MATLAB Version: %s\n', config.metadata.matlab_version);
    end
    fprintf(fid, '================================================================================\n\n');

    % ============================ RUN CONFIG =============================
    fprintf(fid, '--- RUN CONFIGURATION ---\n');
    fprintf(fid, 'Collect Data:                 %d\n', logical_field(config,'run_collect_data',false));
    fprintf(fid, 'Run System ID:                %d\n', logical_field(config,'run_system_id',false));
    fprintf(fid, 'Run MPC:                      %d\n', logical_field(config,'run_MPC',false));
    fprintf(fid, 'Run SPC:                      %d\n', logical_field(config,'run_SPC',false));
    fprintf(fid, 'Run DMC:                      %d\n', logical_field(config,'run_DMC',false));
    fprintf(fid, 'Run DeePC:                    %d\n', logical_field(config,'run_DeePC',false));
    fprintf(fid, '\n');

    % ======================= SIMULATION PARAMETERS =======================
    fprintf(fid, '--- SIMULATION PARAMETERS ---\n');
    fprintf(fid, 'Simulation Time:              %g s\n', config.simulation.t_sim);
    fprintf(fid, 'Sampling Time:                %g s\n', config.simulation.dt);
    fprintf(fid, '\n');

    % ========================= SETPOINT PROFILE ==========================
    fprintf(fid, '--- SETPOINT PROFILE ---\n');
    fprintf(fid, 'Times:                        %s s\n', mat2str(config.setpoint.times));
    fprintf(fid, 'Values:                       %s °C\n', mat2str(config.setpoint.values));
    fprintf(fid, '\n');

    % ======================== CONTROL CONSTRAINTS ========================
    fprintf(fid, '--- CONTROL CONSTRAINTS (SHARED) ---\n');
    fprintf(fid, 'Minimum Input (u_min):        %g %%\n', config.constraints.u_min);
    fprintf(fid, 'Maximum Input (u_max):        %g %%\n', config.constraints.u_max);
    fprintf(fid, 'Max Rate of Change (du_max):  %g %%/sample\n', config.constraints.du_max);
    fprintf(fid, '\n');

    % ================= GLOBAL PREDICTIVE CONTROL =========================
    % Your "current configuration" stores horizons/weights in config.predictive.*
    fprintf(fid, '--- GLOBAL PREDICTIVE CONTROL PARAMETERS ---\n');
    fprintf(fid, 'Prediction Horizon (P):       %d samples\n', config.predictive.P);
    fprintf(fid, 'Control Horizon (M):          %d samples\n', config.predictive.M);
    fprintf(fid, 'Output Tracking Weight (Q):   %.6g\n', config.predictive.Q_weight);
    fprintf(fid, 'Input Change Penalty (R):     %.6g\n', config.predictive.R_weight);
    fprintf(fid, 'Dataset Choice:                  %s\n', config.dataset_choice);
    fprintf(fid, '\n');

    % ============================ MPC ====================================
    % In current config, MPC has enable_integrator only (horizons in predictive)
    fprintf(fid, '--- MPC PARAMETERS ---\n');
    fprintf(fid, 'Enable Integrator:            %d\n', logical_field(config,'enable_integrator',false));
    fprintf(fid, '\n');

    % ============================ SPC ====================================
    fprintf(fid, '--- SPC (SUBSPACE PREDICTIVE CONTROL) PARAMETERS ---\n');
    if isfield(config,'SPC') && isfield(config.SPC,'ident')
        fprintf(fid, 'Identification Method:        %s\n', config.SPC.ident.method);
        fprintf(fid, 'Identified State Dimension:   %d\n', config.SPC.ident.nx);
        fprintf(fid, 'Identification Focus:         %s\n', config.SPC.ident.focus);
        fprintf(fid, 'Identification Form:          %s\n', config.SPC.ident.form);
    else
        fprintf(fid, '(SPC.ident not present)\n');
    end
    fprintf(fid, '\n');

    % ============================ DMC ====================================
    % In current config, DMC includes N and step_response_source only
    fprintf(fid, '--- DMC PARAMETERS ---\n');
    fprintf(fid, 'Model Horizon (N):            %d samples\n', config.DMC.N);
    fprintf(fid, '\n');

    % =========================== DeePC ===================================
    fprintf(fid, '--- DeePC PARAMETERS ---\n');
    fprintf(fid, 'Past Horizon (T_ini):         %d samples\n', config.DeePC.T_ini);
    fprintf(fid, 'Future Horizon (N):           %d samples\n', config.DeePC.N);
    fprintf(fid, 'Slack Penalty Output (lambda_y): %g\n', config.DeePC.lambda_y);
    fprintf(fid, 'Slack Penalty Hankel (lambda_g): %g\n', config.DeePC.lambda_g);
    fprintf(fid, 'Input Regularization (lambda_u): %g\n', config.DeePC.lambda_u);
    fprintf(fid, '\n');

    % ============================ NOISE ==================================
    fprintf(fid, '--- NOISE PARAMETERS ---\n');
    fprintf(fid, 'Noise Enabled:                %d\n', logical_field(config.noise,'enable',false));
    fprintf(fid, 'Noise Type:                   %s\n', config.noise.type);
    fprintf(fid, 'Target SNR:                   %.3g dB\n', config.noise.SNR_dB);
    fprintf(fid, 'Random Seed:                  %d\n', config.noise.seed);
    if strcmpi(config.noise.type, 'colored')
        fprintf(fid, 'Filter Order:                 %d\n', config.noise.colored_filter_order);
        fprintf(fid, 'Cutoff Frequency:             %.3g (normalized)\n', config.noise.colored_cutoff_freq);
    end
    fprintf(fid, '\n');

    % ====================== DATA COLLECTION PARAMETERS ===================
    fprintf(fid, '--- DATA COLLECTION PARAMETERS ---\n');
    fprintf(fid, 'Experiment Duration:          %g s\n', config.data_collection.t_final);
    fprintf(fid, 'Sampling Time:                %g s\n', config.data_collection.dt);
    fprintf(fid, 'Step Amplitude:               %g %%\n', config.data_collection.step_amplitude);
    fprintf(fid, 'Step Delay:                   %g s\n', config.data_collection.step_delay);
    fprintf(fid, 'Impulse Amplitude:            %g %%\n', config.data_collection.impulse_amplitude);
    fprintf(fid, 'Impulse Duration:             %g s\n', config.data_collection.impulse_duration);
    fprintf(fid, 'Impulse Delay:                %g s\n', config.data_collection.impulse_delay);
    fprintf(fid, 'Multisine Amplitude:          %g %%\n', config.data_collection.multisine_amplitude);
    fprintf(fid, 'Multisine Offset:             %g %%\n', config.data_collection.multisine_offset);
    fprintf(fid, 'Frequency Range:              %.6g - %.6g Hz\n', ...
            config.data_collection.freq_min, config.data_collection.freq_max);
    fprintf(fid, 'Number of Frequencies:        %d\n', config.data_collection.n_freq);
    fprintf(fid, 'Doublet Amplitude:            %g %%\n', config.data_collection.doublet_amplitude);
    fprintf(fid, 'Doublet Duration:             %g s\n', config.data_collection.doublet_duration);
    fprintf(fid, 'Doublet Delay:                %g s\n', config.data_collection.doublet_delay);
    fprintf(fid, '\n');

    % =================== SYSTEM IDENTIFICATION PARAMETERS =================
    % Current config does NOT define config.system_id.dataset_choice, so omit it.
    fprintf(fid, '--- SYSTEM IDENTIFICATION PARAMETERS ---\n');
    fprintf(fid, 'Number of Poles:              %d\n', config.system_id.n_poles);
    fprintf(fid, 'Number of Zeros:              %d\n', config.system_id.n_zeros);
    fprintf(fid, 'Validation Fraction:          %.1f%%\n', config.system_id.validation_fraction * 100);
    fprintf(fid, '\n');

    % ======================= THERMAL MODEL PARAMETERS ====================
    fprintf(fid, '--- THERMAL MODEL PARAMETERS ---\n');
    fprintf(fid, 'Initial Temperature (T0):     %g °C\n', config.thermal_model.T0);
    fprintf(fid, 'Ambient Temperature (Ta):     %g °C\n', config.thermal_model.Ta);
    fprintf(fid, 'Heat Transfer Coeff (U):      %g W/m^2-K\n', config.thermal_model.U);
    fprintf(fid, 'Surface Area (A):             %.6g m^2\n', config.thermal_model.A);
    fprintf(fid, 'Mass (m):                     %.6g kg\n', config.thermal_model.m);
    fprintf(fid, 'Heat Capacity (Cp):           %g J/kg-K\n', config.thermal_model.Cp);
    fprintf(fid, 'Heater Efficiency (alpha):    %g\n', config.thermal_model.alpha);
    fprintf(fid, 'Radiation Coeff (eps):        %g\n', config.thermal_model.eps);
    fprintf(fid, 'Stefan-Boltzmann (sigma):     %.6g W/m^2-K^4\n', config.thermal_model.sigma);
    fprintf(fid, 'Model Type:                   %s\n', config.thermal_model.model_type);
    fprintf(fid, '\n');

    % ========================== PLOTTING PARAMETERS =======================
    fprintf(fid, '--- PLOTTING PARAMETERS ---\n');
    if isfield(config,'plotting') && isfield(config.plotting,'colors')
        if isfield(config.plotting.colors,'MPC')
            fprintf(fid, 'MPC Color (RGB):              [%.3g, %.3g, %.3g]\n', config.plotting.colors.MPC);
        end
        if isfield(config.plotting.colors,'SPC')
            fprintf(fid, 'SPC Color (RGB):              [%.3g, %.3g, %.3g]\n', config.plotting.colors.SPC);
        end
        if isfield(config.plotting.colors,'DMC')
            fprintf(fid, 'DMC Color (RGB):              [%.3g, %.3g, %.3g]\n', config.plotting.colors.DMC);
        end
        if isfield(config.plotting.colors,'DeePC')
            fprintf(fid, 'DeePC Color (RGB):            [%.3g, %.3g, %.3g]\n', config.plotting.colors.DeePC);
        end
    end
    fprintf(fid, 'Line Width:                   %g\n', config.plotting.linewidth);
    fprintf(fid, 'Figure Size:                  [%d, %d, %d, %d]\n', ...
            config.plotting.figure_size(1), config.plotting.figure_size(2), ...
            config.plotting.figure_size(3), config.plotting.figure_size(4));
    fprintf(fid, '\n');

    % ============================== FOOTER ===============================
    fprintf(fid, '================================================================================\n');
    fprintf(fid, 'END OF CONFIGURATION\n');
    fprintf(fid, '================================================================================\n');

    fclose(fid);
end

function v = logical_field(s, fieldName, defaultVal)
%LOGICAL_FIELD Safely read a logical-ish field from a struct, else default.
    if isstruct(s) && isfield(s, fieldName)
        v = logical(s.(fieldName));
    else
        v = logical(defaultVal);
    end
end