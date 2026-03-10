function SPC_results = run_closed_loop_spc(config)
%RUN_CLOSED_LOOP_SPC Closed-loop experiment for SPC controller.
%Produces the same SPC_results fields as the original SPC_controller.m.

    if nargin < 1
        config = config_simulation();
    end

    close all;
    if ~exist('results', 'dir'), mkdir('results'); end

    fprintf('========================================\n');
    fprintf('  SPC (Subspace Predictive Control) - Experiment\n');
    fprintf('========================================\n\n');

    % SPC uses current reference only in mpcmove, but keep preview=1
    config.ref_preview_horizon = 1;
    config.controller_name = 'SPC';

    % Build controller policy
    [ctrl_step, ctrl_init, meta] = spc_policy_factory(config);

    % Run generic experiment
    results = run_closed_loop_generic(config, ctrl_step, ctrl_init);

    % Unpack into legacy-shaped struct
    SPC_results = struct();
    SPC_results.t = results.t;
    SPC_results.y = results.y;
    SPC_results.u = results.u;
    SPC_results.setpoint = results.setpoint;
    SPC_results.error = results.error;

    SPC_results.MAE = results.metrics.MAE;
    SPC_results.RMSE = results.metrics.RMSE;
    SPC_results.ISE = results.metrics.ISE;
    SPC_results.IAE = results.metrics.IAE;
    SPC_results.control_effort = results.metrics.control_effort;

    SPC_results.params = struct( ...
        'P', config.SPC.P, ...
        'M', config.SPC.M, ...
        'Q', config.SPC.Q_weight, ...
        'R', config.SPC.R_weight, ...
        'nx', meta.nx);

    % Keep same extra fields as original SPC_controller.m
    tmp = ctrl_init();
    SPC_results.model = meta.model;
    SPC_results.u_mean = meta.u_mean;
    SPC_results.y_mean = meta.y_mean;
    SPC_results.config = config;

    save('results/SPC_results.mat', 'SPC_results');
    fprintf('\nSaved: results/SPC_results.mat\n');
end