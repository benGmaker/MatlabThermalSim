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
    if ~exist('results', 'dir')
        mkdir('results');
    end
    if ~exist('results/config', 'dir')
        mkdir('results/config');
    end
    if ~exist('results/CLI_output', 'dir')
        mkdir('results/CLI_output');
    end
    if ~exist('results/data', 'dir')
        mkdir('results/data');
    end

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
    if config.run_collect_data
        fprintf('STEP 1: Data Collection\n');
        diary('results/CLI_output/step1_data_collection.txt');
        fprintf('Running experiments...\n');
        experiment_data_collection(config);
        diary off;
        fprintf('✓ Complete. Log: results/CLI_output/step1_data_collection.txt\n\n');
        clc;
    else
        fprintf('SKIPPING STEP 1: Data Collection\n')
    end

    %% Step 2: System Identification
    if config.run_system_id
        fprintf('STEP 2: System Identification\n');
        diary('results/CLI_output/step2_system_identification.txt');
        system_identification(config);
        diary off;
        fprintf('✓ Complete. Log: results/CLI_output/step2_system_identification.txt\n\n');
        clc;
    else
        fprintf('SKIPPING STEP 2: System Identifcation\n')
    end


