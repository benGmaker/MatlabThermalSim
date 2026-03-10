function MPC_results = run_closed_loop_mpc(config)

    if nargin < 1, config = config_simulation(); end
    close all;
    if ~exist('results','dir'), mkdir('results'); end

    config.ref_preview_horizon = 1;

    [ctrl_step, ctrl_init] = mpc_policy_factory(config);
    [results, ctrl_final] = run_closed_loop_generic(config, ctrl_step, ctrl_init); %#ok<ASGLU>

    MPC_results = struct();
    MPC_results.t = results.t;
    MPC_results.y = results.y;
    MPC_results.u = results.u;
    MPC_results.setpoint = results.setpoint;
    MPC_results.error = results.error;

    MPC_results.MAE = results.metrics.MAE;
    MPC_results.RMSE = results.metrics.RMSE;
    MPC_results.ISE = results.metrics.ISE;
    MPC_results.IAE = results.metrics.IAE;
    MPC_results.control_effort = results.metrics.control_effort;

    MPC_results.params = struct('P', config.MPC.P, 'M', config.MPC.M, 'Q', config.MPC.Q_weight, 'R', config.MPC.R_weight);

    % keep same as original MPC_controller.m
    tmp = ctrl_init();
    MPC_results.config = config;

    save('results/MPC_results.mat', 'MPC_results');
end