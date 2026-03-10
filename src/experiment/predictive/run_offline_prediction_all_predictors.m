function AllPred_results = run_offline_prediction_all_predictors(config)
%RUN_OFFLINE_PREDICTION_ALL_PREDICTORS
% Runs offline prediction on recorded multisine for multiple predictors and saves one bundle.

    if nargin < 1, config = config_simulation(); end
    if ~exist('results','dir'), mkdir('results'); end

    predictors = {
        struct('name','MPC_sys_ss',   'factory', @mpc_offline_predictor_factory)
        struct('name','SPC_n4sid',    'factory', @spc_offline_predictor_factory)
        struct('name','DMC_step',     'factory', @dmc_offline_predictor_factory)
        struct('name','DeePC_hankel', 'factory', @deepc_offline_predictor_factory)
        
    };

    AllPred_results = struct();
    AllPred_results.config = config;
    AllPred_results.created_at = char(datetime('now'));
    AllPred_results.items = struct([]);

    fprintf('========================================\n');
    fprintf('  Offline Prediction Runs (multisine)\n');
    fprintf('========================================\n');

    for i = 1:numel(predictors)
        p = predictors{i};
        fprintf('\n---- Predictor: %s ----\n', p.name);

        [pred_step, pred_init, meta] = p.factory(config);
        res = run_offline_prediction_multisine_generic(config, pred_step, pred_init);

        item = struct();
        item.name = p.name;
        item.results = res;
        item.meta = meta;

        AllPred_results.items = [AllPred_results.items; item]; %#ok<AGROW>

        fprintf('MAE=%.4f  RMSE=%.4f  fit=%.2f%%\n', ...
            res.metrics.MAE, res.metrics.RMSE, res.metrics.fit_percent);
    end

    save('results/OfflinePred_all_predictors.mat', 'AllPred_results');
    fprintf('\nSaved: results/OfflinePred_all_predictors.mat\n');
end