function DeePC_results = run_closed_loop_deepc(config)

    if nargin < 1
        config = config_simulation();
    end

    close all;
    if ~exist('results', 'dir'), mkdir('results'); end

    config.ref_preview_horizon = config.DeePC.N;
    config.controller_name = 'DeePC';

    [ctrl_step, ctrl_init, meta] = deepc_policy_factory(config);

    [results, ctrl_final] = run_closed_loop_generic(config, ctrl_step, ctrl_init);

    DeePC_results = struct();
    DeePC_results.t = results.t;
    DeePC_results.y = results.y;
    DeePC_results.u = results.u;
    DeePC_results.setpoint = results.setpoint;
    DeePC_results.error = results.error;

    DeePC_results.MAE = results.metrics.MAE;
    DeePC_results.RMSE = results.metrics.RMSE;
    DeePC_results.ISE = results.metrics.ISE;
    DeePC_results.IAE = results.metrics.IAE;
    DeePC_results.control_effort = results.metrics.control_effort;

    DeePC_results.n_failures = ctrl_final.n_failures;

    DeePC_results.params = struct('T_ini', config.DeePC.T_ini, 'N', config.DeePC.N, ...
        'lambda_y', config.DeePC.lambda_y, 'lambda_g', config.DeePC.lambda_g, 'lambda_u', config.DeePC.lambda_u);

    DeePC_results.config = config;

    save('results/DeePC_results.mat', 'DeePC_results');
end