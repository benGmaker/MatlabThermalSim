function run_offline_prediction_test()
%RUN_OFFLINE_PREDICTION_TEST
% Offline predictive-performance pipeline on recorded multisine data.
% Mirrors run_closed_loop_test.m structure:
%   1) Data collection
%   2) System identification
%   3) Offline prediction runs (MPC-model, DMC-step, DeePC, SPC)
%   4) Comparison (tables + plots)

    clc; close all;

    % Keep same path setup style as run_closed_loop_test
    addpath(genpath('src'));
    addpath(genpath('common'));
    addpath(genpath('controllers'));
    addpath(genpath('experiments'));

    % Create results directories
    if ~exist('results', 'dir'), mkdir('results'); end
    if ~exist('results/config', 'dir'), mkdir('results/config'); end
    if ~exist('results/CLI_output', 'dir'), mkdir('results/CLI_output'); end

    % Load centralized configuration
    config = config_simulation();

    % Save configuration in both formats (same as original)
    save('results/config/config_used.mat', 'config');
    save_config_readable(config, 'results/config/config_used.txt');
    fprintf('Configuration loaded and saved to results/config/\n');
    fprintf('  - config_used.mat (MATLAB format)\n');
    fprintf('  - config_used.txt (human-readable)\n\n');

    fprintf('========================================\n');
    fprintf('  Offline Prediction Evaluation\n');
    fprintf('========================================\n');
    fprintf('Start Time: %s\n', char(datetime('now')));
    fprintf('Configuration: See results/config/\n');
    fprintf('========================================\n\n');

    %% Step 1: Data Collection
    fprintf('STEP 1: Data Collection\n');
    diary('results/CLI_output/offline_step1_data_collection.txt');
    fprintf('Running experiments...\n');
    experiment_data_collection(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/offline_step1_data_collection.txt\n\n');
    clc;

    %% Step 2: System Identification
    fprintf('STEP 2: System Identification\n');
    diary('results/CLI_output/offline_step2_system_identification.txt');
    system_identification(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/offline_step2_system_identification.txt\n\n');
    clc;

    %% Step 3: Offline Prediction (all predictors)
    fprintf('STEP 3: Offline Prediction Runs\n');
    diary('results/CLI_output/offline_step3_prediction_runs.txt');
    run_offline_prediction_all_predictors(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/offline_step3_prediction_runs.txt\n\n');
    clc;

    %% Step 4: Comparison
    fprintf('STEP 4: Predictor Comparison\n');
    diary('results/CLI_output/offline_step4_comparison.txt');
    compare_predictors(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/offline_step4_comparison.txt\n\n');

    %% Final Summary
    fprintf('\n========================================\n');
    fprintf('  Offline Prediction Pipeline Complete!\n');
    fprintf('========================================\n');
    fprintf('End Time: %s\n', char(datetime('now')));
    fprintf('\nResults saved in:\n');
    fprintf('  - results/OfflinePred_all_predictors.mat\n');
    fprintf('  - results/predictor_comparison.png\n');
    fprintf('  - results/predictor_error_distributions.png\n');
    fprintf('  - results/CLI_output/*offline*.txt\n');
    fprintf('========================================\n');
end