function run_closed_loop_test()
% RUN_CLOSED_LOOP_TEST Compare DMC, MPC, and DeePC controller performance

    clc; close all;
    
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    fprintf('========================================\n');
    fprintf('  Thermal Control System Simulation\n');
    fprintf('========================================\n\n');
    
    %% Step 1: Data Collection
    fprintf('STEP 1: Data Collection\n');
    fprintf('Running experiments...\n');
    experiment_data_collection();
    fprintf('\n');
    
    %% Step 2: System Identification
    fprintf('STEP 2: System Identification\n');
    system_identification();
    fprintf('\n');
    
    %% Step 3: DMC Controller
    fprintf('STEP 3: DMC Controller Simulation\n');
    DMC_controller();
    fprintf('\n');
    
    %% Step 4: MPC Controller
    fprintf('STEP 4: MPC Controller Simulation\n');
    MPC_controller();
    fprintf('\n');
    
    %% Step 5: DeePC Controller
    fprintf('STEP 5: DeePC Controller Simulation\n');
    DeePC_controller();
    fprintf('\n');
    
    %% Step 6: Comparison
    fprintf('STEP 6: Controller Comparison\n');
    compare_controllers();
    
    fprintf('\n========================================\n');
    fprintf('  Simulation Complete!\n');
    fprintf('========================================\n');
end

function compare_controllers()
% Compare DMC, MPC, and DeePC controller performance
% IMPROVED: Consistent colors and ordering (MPC, DMC, DeePC)
    
    % Load results
    load('results/DMC_results.mat', 'DMC_results');
    load('results/MPC_results.mat', 'MPC_results');
    load('results/DeePC_results.mat', 'DeePC_results');
    
    % Define consistent colors and ordering
    % Order: MPC, DMC, DeePC
    % Using lighter, more vibrant colors for better contrast
    colors = struct();
    colors.MPC = [0.2, 0.7, 0.2];      % Lighter Green
    colors.DMC = [0.2, 0.4, 0.9];      % Lighter Blue
    colors.DeePC = [0.9, 0.2, 0.2];    % Lighter Red
    
    controller_order = {'MPC', 'DMC', 'DeePC'};
    controller_labels = {'MPC', 'DMC', 'DeePC'};
    
    %% Print comparison table
    fprintf('\n========== Performance Comparison ==========\n');
    fprintf('%-20s %-15s %-15s %-15s\n', 'Metric', 'MPC', 'DMC', 'DeePC');
    fprintf('%-20s %-15s %-15s %-15s\n', repmat('-', 1, 20), repmat('-', 1, 15), ...
            repmat('-', 1, 15), repmat('-', 1, 15));
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'MAE [°C]', ...
            MPC_results.MAE, DMC_results.MAE, DeePC_results.MAE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'RMSE [°C]', ...
            MPC_results.RMSE, DMC_results.RMSE, DeePC_results.RMSE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'ISE', ...
            MPC_results.ISE, DMC_results.ISE, DeePC_results.ISE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'IAE', ...
            MPC_results.IAE, DMC_results.IAE, DeePC_results.IAE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'Control Effort', ...
            MPC_results.control_effort, DMC_results.control_effort, DeePC_results.control_effort);
    fprintf('============================================\n\n');
    
    % Find best performer for each metric
    [~, best_mae] = min([MPC_results.MAE, DMC_results.MAE, DeePC_results.MAE]);
    [~, best_rmse] = min([MPC_results.RMSE, DMC_results.RMSE, DeePC_results.RMSE]);
    [~, best_ise] = min([MPC_results.ISE, DMC_results.ISE, DeePC_results.ISE]);
    
    fprintf('Best MAE:  %s (%.3f °C)\n', controller_order{best_mae}, ...
            min([MPC_results.MAE, DMC_results.MAE, DeePC_results.MAE]));
    fprintf('Best RMSE: %s (%.3f °C)\n', controller_order{best_rmse}, ...
            min([MPC_results.RMSE, DMC_results.RMSE, DeePC_results.RMSE]));
    fprintf('Best ISE:  %s (%.3f)\n', controller_order{best_ise}, ...
            min([MPC_results.ISE, DMC_results.ISE, DeePC_results.ISE]));
    
    %% Comparison plots
    figure('Position', [100, 100, 1600, 1000]);
    
    % Temperature tracking
    subplot(3,3,1);
    plot(MPC_results.t, MPC_results.setpoint, 'k--', 'LineWidth', 2); hold on;
    plot(MPC_results.t, MPC_results.y, 'Color', colors.MPC, 'LineWidth', 1.5);
    plot(DMC_results.t, DMC_results.y, 'Color', colors.DMC, 'LineWidth', 1.5);
    plot(DeePC_results.t, DeePC_results.y, 'Color', colors.DeePC, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'MPC', 'DMC', 'DeePC', 'Location', 'best');
    title('Temperature Tracking Comparison');
    grid on;
    
    % Control inputs
    subplot(3,3,2);
    stairs(MPC_results.t, MPC_results.u, 'Color', colors.MPC, 'LineWidth', 1.5); hold on;
    stairs(DMC_results.t, DMC_results.u, 'Color', colors.DMC, 'LineWidth', 1.5);
    stairs(DeePC_results.t, DeePC_results.u, 'Color', colors.DeePC, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    legend('MPC', 'DMC', 'DeePC', 'Location', 'best');
    title('Control Input Comparison');
    grid on;
    
    % Tracking errors
    subplot(3,3,3);
    plot(MPC_results.t, abs(MPC_results.error), 'Color', colors.MPC, 'LineWidth', 1.5); hold on;
    plot(DMC_results.t, abs(DMC_results.error), 'Color', colors.DMC, 'LineWidth', 1.5);
    plot(DeePC_results.t, abs(DeePC_results.error), 'Color', colors.DeePC, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Absolute Error [°C]');
    legend('MPC', 'DMC', 'DeePC', 'Location', 'best');
    title('Absolute Tracking Error');
    grid on;
    
    % Zoom in: First setpoint change (150-250s)
    subplot(3,3,4);
    idx = MPC_results.t >= 150 & MPC_results.t <= 250;
    plot(MPC_results.t(idx), MPC_results.setpoint(idx), 'k--', 'LineWidth', 2); hold on;
    plot(MPC_results.t(idx), MPC_results.y(idx), 'Color', colors.MPC, 'LineWidth', 1.5);
    plot(DMC_results.t(idx), DMC_results.y(idx), 'Color', colors.DMC, 'LineWidth', 1.5);
    plot(DeePC_results.t(idx), DeePC_results.y(idx), 'Color', colors.DeePC, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Zoom: First Setpoint Change');
    grid on;
    
    % Zoom in: Second setpoint change (350-450s)
    subplot(3,3,5);
    idx = MPC_results.t >= 350 & MPC_results.t <= 450;
    plot(MPC_results.t(idx), MPC_results.setpoint(idx), 'k--', 'LineWidth', 2); hold on;
    plot(MPC_results.t(idx), MPC_results.y(idx), 'Color', colors.MPC, 'LineWidth', 1.5);
    plot(DMC_results.t(idx), DMC_results.y(idx), 'Color', colors.DMC, 'LineWidth', 1.5);
    plot(DeePC_results.t(idx), DeePC_results.y(idx), 'Color', colors.DeePC, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Zoom: Second Setpoint Change');
    grid on;
    
    % Control effort comparison (rate of change)
    subplot(3,3,6);
    du_mpc = [0; diff(MPC_results.u)];
    du_dmc = [0; diff(DMC_results.u)];
    du_deepc = [0; diff(DeePC_results.u)];
    plot(MPC_results.t, abs(du_mpc), 'Color', colors.MPC, 'LineWidth', 1.5); hold on;
    plot(DMC_results.t, abs(du_dmc), 'Color', colors.DMC, 'LineWidth', 1.5);
    plot(DeePC_results.t, abs(du_deepc), 'Color', colors.DeePC, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('|du| [%/sample]');
    legend('MPC', 'DMC', 'DeePC', 'Location', 'best');
    title('Control Rate of Change');
    grid on;
    
    % Performance metrics bar chart - CORRECTED
    % X-axis: Metrics (MAE, RMSE)
    % Bars: Three bars per metric (MPC, DMC, DeePC)
    subplot(3,3,7);
    metrics_data = [MPC_results.MAE, MPC_results.RMSE; ...
                    DMC_results.MAE, DMC_results.RMSE; ...
                    DeePC_results.MAE, DeePC_results.RMSE];
    
    b = bar(metrics_data');
    b(1).FaceColor = colors.MPC;
    b(2).FaceColor = colors.DMC;
    b(3).FaceColor = colors.DeePC;
    
    set(gca, 'XTickLabel', {'MAE', 'RMSE'});
    ylabel('Error [°C]');
    legend('MPC', 'DMC', 'DeePC', 'Location', 'best');
    title('Tracking Error Metrics');
    grid on;
    
    % ISE and IAE comparison - CORRECTED
    % X-axis: Metrics (ISE, IAE)
    % Bars: Three bars per metric (MPC, DMC, DeePC)
    subplot(3,3,8);
    integrated_data = [MPC_results.ISE, MPC_results.IAE; ...
                       DMC_results.ISE, DMC_results.IAE; ...
                       DeePC_results.ISE, DeePC_results.IAE];
    
    b = bar(integrated_data');
    b(1).FaceColor = colors.MPC;
    b(2).FaceColor = colors.DMC;
    b(3).FaceColor = colors.DeePC;
    
    set(gca, 'XTickLabel', {'ISE', 'IAE'});
    ylabel('Integrated Error');
    legend('MPC', 'DMC', 'DeePC', 'Location', 'best');
    title('Integrated Error Metrics');
    grid on;
    
    % Control effort comparison - CORRECT (with colors)
    subplot(3,3,9);
    control_efforts = [MPC_results.control_effort; ...
                       DMC_results.control_effort; ...
                       DeePC_results.control_effort];
    b = bar(control_efforts);
    b.FaceColor = 'flat';
    b.CData = [colors.MPC; colors.DMC; colors.DeePC];
    set(gca, 'XTickLabel', controller_labels);
    ylabel('Total Control Effort');
    title('Control Effort Comparison');
    grid on;
    
    sgtitle('Controller Performance Comparison: MPC vs DMC vs DeePC');
    saveas(gcf, 'results/controller_comparison.png');
    fprintf('  Saved: results/controller_comparison.png\n');
    
    %% Statistical analysis (IMPROVED COLORS)
    figure('Position', [100, 100, 1200, 400]);
    
    % Error distribution
    subplot(1,3,1);
    h1 = histogram(MPC_results.error, 30, 'Normalization', 'probability', ...
              'FaceColor', colors.MPC, 'FaceAlpha', 0.5, 'DisplayName', 'MPC'); hold on;
    h2 = histogram(DMC_results.error, 30, 'Normalization', 'probability', ...
              'FaceColor', colors.DMC, 'FaceAlpha', 0.5, 'DisplayName', 'DMC');
    h3 = histogram(DeePC_results.error, 30, 'Normalization', 'probability', ...
              'FaceColor', colors.DeePC, 'FaceAlpha', 0.5, 'DisplayName', 'DeePC');
    xlabel('Tracking Error [°C]');
    ylabel('Probability');
    legend('Location', 'best');
    title('Error Distribution');
    grid on;
    
    % Cumulative error
    subplot(1,3,2);
    plot(MPC_results.t, cumsum(abs(MPC_results.error))*MPC_results.t(2), ...
         'Color', colors.MPC, 'LineWidth', 1.5); hold on;
    plot(DMC_results.t, cumsum(abs(DMC_results.error))*DMC_results.t(2), ...
         'Color', colors.DMC, 'LineWidth', 1.5);
    plot(DeePC_results.t, cumsum(abs(DeePC_results.error))*DeePC_results.t(2), ...
         'Color', colors.DeePC, 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Cumulative Absolute Error');
    legend('MPC', 'DMC', 'DeePC', 'Location', 'best');
    title('Cumulative Tracking Error');
    grid on;
    
    % Settling time analysis (approximate)
    subplot(1,3,3);
    threshold = 0.5;  % 0.5°C error threshold
    settling_mpc = compute_settling_metrics(MPC_results.t, MPC_results.error, threshold);
    settling_dmc = compute_settling_metrics(DMC_results.t, DMC_results.error, threshold);
    settling_deepc = compute_settling_metrics(DeePC_results.t, DeePC_results.error, threshold);
    
    settling_values = [settling_mpc.mean; settling_dmc.mean; settling_deepc.mean];
    b = bar(settling_values);
    b.FaceColor = 'flat';
    b.CData = [colors.MPC; colors.DMC; colors.DeePC];
    set(gca, 'XTickLabel', controller_labels);
    ylabel('Average Settling Time [s]');
    title(sprintf('Settling Time (±%.1f°C threshold)', threshold));
    grid on;
    
    sgtitle('Statistical Analysis of Controller Performance');
    saveas(gcf, 'results/statistical_analysis.png');
    fprintf('  Saved: results/statistical_analysis.png\n');
    
    %% Save comparison summary
    comparison_summary = struct();
    comparison_summary.MPC = MPC_results;
    comparison_summary.DMC = DMC_results;
    comparison_summary.DeePC = DeePC_results;
    comparison_summary.best_mae = controller_order{best_mae};
    comparison_summary.best_rmse = controller_order{best_rmse};
    comparison_summary.best_ise = controller_order{best_ise};
    comparison_summary.colors = colors;
    comparison_summary.controller_order = controller_order;
    
    save('results/comparison_summary.mat', 'comparison_summary');
    fprintf('  Saved: results/comparison_summary.mat\n');
end

function metrics = compute_settling_metrics(t, error, threshold)
% Compute settling time metrics for setpoint changes
    
    % Find setpoint change indices (approximate)
    changes = [200, 400] / (t(2) - t(1));  % Convert to indices
    settling_times = [];
    
    for i = 1:length(changes)
        idx_start = round(changes(i));
        if idx_start > length(error)
            continue;
        end
        
        % Find when error stays within threshold
        idx_end = idx_start;
        while idx_end < length(error)
            if all(abs(error(idx_end:min(idx_end+10, length(error)))) < threshold)
                settling_times = [settling_times, (idx_end - idx_start) * (t(2) - t(1))];
                break;
            end
            idx_end = idx_end + 1;
        end
    end
    
    if isempty(settling_times)
        metrics.mean = NaN;
        metrics.max = NaN;
    else
        metrics.mean = mean(settling_times);
        metrics.max = max(settling_times);
    end
end