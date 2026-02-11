function run_closed_loop_test()
% RUN_CLOSED_LOOP_TEST Compare DMC, MPC, and DeePC controller performance
% Now with centralized configuration and CLI output logging
    
    clc; close all;
    addpath(genpath('src')) % loading source code

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
    fprintf('STEP 1: Data Collection\n');
    diary('results/CLI_output/step1_data_collection.txt');
    fprintf('Running experiments...\n');
    experiment_data_collection(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/step1_data_collection.txt\n\n');
    clc;
    
    %% Step 2: System Identification
    fprintf('STEP 2: System Identification\n');
    diary('results/CLI_output/step2_system_identification.txt');
    system_identification(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/step2_system_identification.txt\n\n');
    clc;
    
    %% Step 3: DMC Controller
    fprintf('STEP 3: DMC Controller Simulation\n');
    diary('results/CLI_output/step3_DMC_controller.txt');
    DMC_controller(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/step3_DMC_controller.txt\n\n');
    clc;
    
    %% Step 4: MPC Controller
    fprintf('STEP 4: MPC Controller Simulation\n');
    diary('results/CLI_output/step4_MPC_controller.txt');
    MPC_controller(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/step4_MPC_controller.txt\n\n');
    clc;
    
    %% Step 5: DeePC Controller
    fprintf('STEP 5: DeePC Controller Simulation\n');
    diary('results/CLI_output/step5_DeePC_controller.txt');
    DeePC_controller(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/step5_DeePC_controller.txt\n\n');
    clc;
    
    %% Step 6: Comparison
    fprintf('STEP 6: Controller Comparison\n');
    diary('results/CLI_output/step6_comparison.txt');
    compare_controllers(config);
    diary off;
    fprintf('✓ Complete. Log: results/CLI_output/step6_comparison.txt\n\n');
    
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
