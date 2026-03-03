function DMC_results = run_closed_loop_dmc(config)

    if nargin < 1, config = config_simulation(); end
    close all;
    if ~exist('results','dir'), mkdir('results'); end

    config.ref_preview_horizon = config.DMC.P;
    config.controller_name = 'DMC';

    [ctrl_step, ctrl_init, meta] = dmc_policy_factory(config);

    [results, ~] = run_closed_loop_generic(config, ctrl_step, ctrl_init);

    DMC_results = struct();
    DMC_results.t = results.t;
    DMC_results.y = results.y;
    DMC_results.u = results.u;
    DMC_results.setpoint = results.setpoint;
    DMC_results.error = results.error;

    DMC_results.MAE = results.metrics.MAE;
    DMC_results.RMSE = results.metrics.RMSE;
    DMC_results.ISE = results.metrics.ISE;
    DMC_results.IAE = results.metrics.IAE;
    DMC_results.control_effort = results.metrics.control_effort;

    DMC_results.params = struct('P', config.DMC.P, 'M', config.DMC.M, 'N', config.DMC.N, ...
        'Q', config.DMC.Q_weight, 'R', config.DMC.R_weight);

    DMC_results.step_response_source = config.DMC.step_response_source;
    DMC_results.step_response = meta.S;
    DMC_results.config = config;

    save('results/DMC_results.mat', 'DMC_results');
end