function AllPred_results = run_offline_prediction_all_predictors(config)
%RUN_OFFLINE_PREDICTION_ALL_PREDICTORS
% Runs offline prediction on recorded multisine for multiple predictors.

    if nargin < 1, config = config_simulation(); end
    close all;
    if ~exist('results','dir'), mkdir('results'); end

    predictors = {
        struct('name','identified_sys_ss', 'factory', @identified_ss_offline_predictor_factory)
        struct('name','spc_n4sid_ss',      'factory', @spc_offline_predictor_factory)
        struct('name','deepc_hankel',      'factory', @deepc_offline_predictor_factory)
    };

    AllPred_results = struct();
    AllPred_results.config = config;
    AllPred_results.items = struct([]);

    for i = 1:numel(predictors)
        p = predictors{i};
        fprintf('\n---- Offline predictor: %s ----\n', p.name);

        [pred_step, pred_init, meta] = p.factory(config);
        res = run_offline_prediction_multisine_generic(config, pred_step, pred_init);

        item = struct();
        item.name = p.name;
        item.results = res;
        item.meta = meta;

        AllPred_results.items = [AllPred_results.items; item]; %#ok<AGROW>

        fprintf('RMSE=%.4g  MAE=%.4g  fit=%.2f%%\n', ...
            res.metrics.RMSE, res.metrics.MAE, res.metrics.fit_percent);
    end

    save('results/OfflinePred_all_predictors.mat', 'AllPred_results');
    fprintf('\nSaved: results/OfflinePred_all_predictors.mat\n');
end