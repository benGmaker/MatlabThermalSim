function config = config_simulation()
% CONFIG_SIMULATION Centralized configuration for thermal control simulation
% All experimental parameters, controller settings, and constraints in one place
%
% Usage:
%   config = config_simulation();
%   
% The configuration is automatically saved with results for traceability

    %%  ========== RUN CONFIGURATION ==========
    % Closed loop
    config.run_collect_data = false;
    config.run_system_id = false;
    config.run_MPC = false;
    config.run_SPC = true;
    config.run_DMC = false;
    config.run_DeePC = true;
    config.run_closed_loop_comparison = true;

    % Predictive 
    config.run_predictive = false;
    config.run_predictive_comparison = false;

    %% ========== SIMULATION PARAMETERS ==========
    config.simulation.t_sim = 600;          % Simulation time [s]
    config.simulation.dt = 1;               % Sampling time [s]
    
    %% ========== SETPOINT PROFILE ==========
    % Define setpoint changes as [time, temperature] pairs
    config.setpoint.times = [0, 200, 400];      % Setpoint change times [s]
    config.setpoint.values = [40, 50, 35];      % Setpoint temperatures [°C]
    
    %% ========== CONTROL CONSTRAINTS (SHARED ACROSS ALL CONTROLLERS) ==========
    config.constraints.u_min = 0;           % Min heater power [%]
    config.constraints.u_max = 100;         % Max heater power [%]

    % y constraints
    config.constraints.enable_y_constraints = false; 
    config.constraints.y_min = -inf;
    config.constraints.y_max = inf;

    % Du constraints
    config.constraints.enable_du_constraints = false; 
    config.constraints.du_max = inf;         % Max rate of change [%/sample]
    config.constraints.du_min = -inf;
    
    % smoothing (penalty). Start here:
    config.constraints.enable_du_penalty = false;
    config.constraints.du_weight = 0;   

    %% ========== GLOBAL PREDICTIVE CONTROL PARAMETERS ==========
    config.predictive.P = 20;                      % Prediction horizon [samples]
    config.predictive.M = 20;                      % Control horizon [samples]
    config.predictive.Q_weight = 10;                % Output tracking weight
    config.predictive.R_weight = 0.01;            % Input change penalty
    config.dataset_choice = 'doublet';  % options: step, multisine, impulse, doublet 

    % DMC always uses step response data 
    %% ========== MPC PARAMETERS ==========
    config.enable_integrator = false;       % Offset free MPC

    %% ========== SPC (Subspace Predictive Control) PARAMETERS ==========
    config.SPC.ident.nx = 2; % Order of the system

    % Hankel Block size 
    config.SPC.M = 1;  % Past samples data length
    
    %% ========== DMC PARAMETERS ==========
    
    
    %% ========== DeePC PARAMETERS ==========
    config.DeePC.T_ini = 1;                 % Past horizon [samples]
    config.DeePC.lambda_y = 1000;           % Slack penalty (output constraint)
    config.DeePC.lambda_g = 1000;           % Slack penalty (Hankel constraint)
    config.DeePC.lambda_u = 0.1;             % Input regularization
    
    %% ========== NOISE PARAMETERS ==========
    % Noise is added to measurement data during data collection
    config.noise.enable = false;                 % Enable/disable noise
    config.noise.type = 'uniform';             % 'gaussian', 'uniform', or 'colored'
    config.noise.SNR_dB = 10;                   % Desired Signal-to-Noise Ratio [dB]
    config.noise.seed = 66;                     % Random seed (for reproducibility)
    
    % For colored noise only
    config.noise.colored_filter_order = 2;      % Filter order for colored noise
    config.noise.colored_cutoff_freq = 0.1;     % Normalized cutoff frequency [0-1]
    
    %% ========== DATA COLLECTION PARAMETERS ==========
    config.data_collection.t_final = 600;           % Experiment duration [s]
    config.data_collection.dt = 1;                  % Sampling time [s]
    config.data_collection.step_amplitude = 50;     % Step input amplitude [%]
    config.data_collection.step_delay = 2;         % Delay before step [s]
    config.data_collection.impulse_amplitude = 100; % Impulse amplitude [%]
    config.data_collection.impulse_duration = 10;   % Impulse duration [s]
    config.data_collection.impulse_delay = 2;      % Delay before impulse [s]
    config.data_collection.multisine_amplitude = 30;% Multisine amplitude [%]
    config.data_collection.multisine_offset = 20;   % Multisine DC offset [%]
    config.data_collection.freq_min = 0.001;        % Min frequency [Hz]
    config.data_collection.freq_max = 0.05;         % Max frequency [Hz]
    config.data_collection.n_freq = 10;             % Number of frequencies
    config.data_collection.doublet_amplitude = 40;  % Doublet amplitude [%]
    config.data_collection.doublet_duration = 100;  % Duration of each pulse [s]
    config.data_collection.doublet_delay = 2;      % Initial delay [s]
    
    %% ========== SYSTEM IDENTIFICATION PARAMETERS ==========
    config.system_id.n_poles = 2;                   % Number of poles
    config.system_id.n_zeros = 1;                   % Number of zeros
    config.system_id.validation_fraction = 0.3;     % Validation data fraction
    
    %% ========== THERMAL MODEL PARAMETERS ==========
    config.thermal_model.T0 = 23;           % Initial temperature [°C]
    config.thermal_model.Ta = 23;           % Ambient temperature [°C]
    config.thermal_model.U = 10;            % Heat transfer coefficient [W/m^2-K]
    config.thermal_model.A = 1e-3;          % Surface area [m^2]
    config.thermal_model.m = 4e-3;          % Mass [kg]
    config.thermal_model.Cp = 500;          % Heat capacity [J/kg-K]
    config.thermal_model.alpha = 0.01;      % Heater efficiency
    config.thermal_model.eps = 0.9;         % Radiation coefficient
    config.thermal_model.sigma = 5.67e-8;   % Stefan-Boltzmann constant [W/m^2-K^4]
    config.thermal_model.model_type = 'linear';  % 'linear' or 'nonlinear'
    
    %% ========== PLOTTING PARAMETERS ==========
    config.plotting.colors.MPC = [0.2, 0.7, 0.2];      % Lighter Green
    config.plotting.colors.DMC = [0.2, 0.4, 0.9];      % Lighter Blue
    config.plotting.colors.DeePC = [0.9, 0.2, 0.2];    % Lighter Red
    config.plotting.linewidth = 1.5;
    config.plotting.figure_size = [100, 100, 1600, 1000];
    
    %% ========== METADATA ==========
    config.metadata.created = datetime('now');
    config.metadata.matlab_version = version;
    config.metadata.description = 'Centralized configuration for thermal control system simulation';
    
    %% ========== HELPER: PRINT CONFIGURATION ==========
    % Optionally display configuration summary when created
    % Uncomment to enable:
    % print_config_summary(config);
end

function print_config_summary(config)
% Print a summary of the configuration
    fprintf('\n========== Configuration Summary ==========\n');
    fprintf('Simulation Time: %d s\n', config.simulation.t_sim);
    fprintf('Sampling Time: %.1f s\n', config.simulation.dt);
    fprintf('Control Constraints: u ∈ [%d, %d]%%, Δu ≤ %d%%\n', ...
            config.constraints.u_min, config.constraints.u_max, config.constraints.du_max);
    fprintf('Setpoint Profile: %s °C at times %s s\n', ...
            mat2str(config.setpoint.values), mat2str(config.setpoint.times));
    fprintf('Noise: %s, Type: %s, SNR: %.1f dB\n', ...
            char(config.noise.enable), config.noise.type, config.noise.SNR_dB);
    fprintf('MPC Horizons: P=%d, M=%d\n', config.MPC.P, config.MPC.M);
    fprintf('DMC Horizons: P=%d, M=%d, N=%d\n', config.DMC.P, config.DMC.M, config.DMC.N);
    fprintf('DeePC Horizons: T_ini=%d, N=%d\n', config.DeePC.T_ini, config.DeePC.N);
    fprintf('===========================================\n\n');
end