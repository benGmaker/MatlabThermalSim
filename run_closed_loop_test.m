
function run_closed_loop_test()
% RUN_CLOSED_LOOP_TEST Compare DMC, MPC, DeePC, and SPC controller performance
% UPDATED: calls the new experiment runners (experiments/run_closed_loop_*.m)
% and keeps the same CLI logging structure.

    clc; close all;
   
    % If you previously relied on src/, keep it. Also add the new folders.
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

    % Load centralized configuration
    config = config_simulation();

    % Save configuration in both formats
    save('results/config/config_used.mat', 'config');  % MATLAB format (easy loading)
    save_config_readable(config, 'results/config/config_used.txt');  % Text format (human readable)
    fprintf('Configuration loaded and saved to results/config/\n');
    fprintf('  - config_used.mat (MATLAB format)\n');
    fprintf('  - config_used.txt (human-readable)\n\n');

    fprintf('========================================\n');
    fprintf('  Thermal Control System Simulation\n');
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

    %% Step 3: MPC Controller 
    if config.run_MPC
        fprintf('STEP 3: MPC Controller Simulation\n');
        diary('results/CLI_output/step4_MPC_controller.txt');
        run_closed_loop_mpc(config);
        diary off;
        fprintf('✓ Complete. Log: results/CLI_output/step4_MPC_controller.txt\n\n');
        clc;
    else
        fprintf('SKIPPING STEP 3: MPC Controller Simulation\n')
    end

    %% Step 4: SPC Controller 
    if config.run_SPC
        fprintf('STEP 4: SPC Controller Simulation\n');
        diary('results/CLI_output/step6_SPC_controller.txt');
        run_closed_loop_spc(config);
        diary off;
        fprintf('✓ Complete. Log: results/CLI_output/step6_SPC_controller.txt\n\n');
        clc;
    else
        fprintf("SKIPPING STEP 4: SPC Controller Simulation\n")
    end 
    %% Step 5: DMC Controller 
    if config.run_DMC
        fprintf('STEP 5: DMC Controller Simulation\n');
        diary('results/CLI_output/step3_DMC_controller.txt');
        run_closed_loop_dmc(config);
        diary off;
        fprintf('✓ Complete. Log: results/CLI_output/step3_DMC_controller.txt\n\n');
        clc;
    else
        fprintf('SKIPPING STEP 5: DMC Controller Simulation\n')
    end

    %% Step 6: DeePC Controller 
    if config.run_DeePC
        fprintf('STEP 6: DeePC Controller Simulation\n');
        diary('results/CLI_output/step5_DeePC_controller.txt');
        run_closed_loop_deepc(config);
        diary off;
        fprintf('✓ Complete. Log: results/CLI_output/step5_DeePC_controller.txt\n\n');
        %clc;
    else
        fprintf('SKIPPING STEP 6: DeePC Controller Simulation\n')
    end

    %% Step 7: Comparison
    fprintf('STEP 7: Controller Comparison\n');
    diary('results/CLI_output/step7_comparison.txt');
    compare_controllers(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/step7_comparison.txt\n\n');

    %% Final Summary
    fprintf('\n========================================\n');
    fprintf('  Simulation Complete!\n');
    fprintf('========================================\n');
    fprintf('End Time: %s\n', char(datetime('now')));
    fprintf('\nResults saved in:\n');
    fprintf('  - results/*.mat (controller results)\n');
    fprintf('  - results/config/ (configuration used)\n');
    fprintf('  - results/CLI_output/ (command line logs)\n');
    fprintf('  - results/*.png (plots)\n');
    fprintf('========================================\n');
end