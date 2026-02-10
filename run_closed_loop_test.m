function run_closed_loop_test()
% RUN_CLOSED_LOOP_TEST Compare DMC, MPC, and DeePC controller performance

    clc; close all;
    
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
    
    % Load results
    load('DMC_results.mat', 'DMC_results');
    load('MPC_results.mat', 'MPC_results');
    load('DeePC_results.mat', 'DeePC_results');
    
    %% Print comparison table
    fprintf('\n========== Performance Comparison ==========\n');
    fprintf('%-20s %-15s %-15s %-15s\n', 'Metric', 'DMC', 'MPC', 'DeePC');
    fprintf('%-20s %-15s %-15s %-15s\n', repmat('-', 1, 20), repmat('-', 1, 15), ...
            repmat('-', 1, 15), repmat('-', 1, 15));
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'MAE [°C]', ...
            DMC_results.MAE, MPC_results.MAE, DeePC_results.MAE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'RMSE [°C]', ...
            DMC_results.RMSE, MPC_results.RMSE, DeePC_results.RMSE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'ISE', ...
            DMC_results.ISE, MPC_results.ISE, DeePC_results.ISE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'IAE', ...
            DMC_results.IAE, MPC_results.IAE, DeePC_results.IAE);
    fprintf('%-20s %-15.3f %-15.3f %-15.3f\n', 'Control Effort', ...
            DMC_results.control_effort, MPC_results.control_effort, DeePC_results.control_effort);
    fprintf('============================================\n\n');
    
    % Find best performer for each metric
    [~, best_mae] = min([DMC_results.MAE, MPC_results.MAE, DeePC_results.MAE]);
    [~, best_rmse] = min([DMC_results.RMSE, MPC_results.RMSE, DeePC_results.RMSE]);
    [~, best_ise] = min([DMC_results.ISE, MPC_results.ISE, DeePC_results.ISE]);
    
    controllers = {'DMC', 'MPC', 'DeePC'};
    fprintf('Best MAE:  %s (%.3f °C)\n', controllers{best_mae}, ...
            min([DMC_results.MAE, MPC_results.MAE, DeePC_results.MAE]));
    fprintf('Best RMSE: %s (%.3f °C)\n', controllers{best_rmse}, ...
            min([DMC_results.RMSE, MPC_results.RMSE, DeePC_results.RMSE]));
    fprintf('Best ISE:  %s (%.3f)\n', controllers{best_ise}, ...
            min([DMC_results.ISE, MPC_results.ISE, DeePC_results.ISE]));
    
    %% Comparison plots
    figure('Position', [100, 100, 1600, 1000]);
    
    % Temperature tracking
    subplot(3,3,1);
    plot(DMC_results.t, DMC_results.setpoint, 'k--', 'LineWidth', 2); hold on;
    plot(DMC_results.t, DMC_results.y, 'b', 'LineWidth', 1.5);
    plot(MPC_results.t, MPC_results.y, 'g', 'LineWidth', 1.5);
    plot(DeePC_results.t, DeePC_results.y, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'DMC', 'MPC', 'DeePC', 'Location', 'best');
    title('Temperature Tracking Comparison');
    grid on;
    
    % Control inputs
    subplot(3,3,2);
    stairs(DMC_results.t, DMC_results.u, 'b', 'LineWidth', 1.5); hold on;
    stairs(MPC_results.t, MPC_results.u, 'g', 'LineWidth', 1.5);
    stairs(DeePC_results.t, DeePC_results.u, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    legend('DMC', 'MPC', 'DeePC', 'Location', 'best');
    title('Control Input Comparison');
    grid on;
    
    % Tracking errors
    subplot(3,3,3);
    plot(DMC_results.t, abs(DMC_results.error), 'b', 'LineWidth', 1.5); hold on;
    plot(MPC_results.t, abs(MPC_results.error), 'g', 'LineWidth', 1.5);
    plot(DeePC_results.t, abs(DeePC_results.error), 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Absolute Error [°C]');
    legend('DMC', 'MPC', 'DeePC', 'Location', 'best');
    title('Absolute Tracking Error');
    grid on;
    
    % Zoom in: First setpoint change (150-250s)
    subplot(3,3,4);
    idx = DMC_results.t >= 150 & DMC_results.t <= 250;
    plot(DMC_results.t(idx), DMC_results.setpoint(idx), 'k--', 'LineWidth', 2); hold on;
    plot(DMC_results.t(idx), DMC_results.y(idx), 'b', 'LineWidth', 1.5);
    plot(MPC_results.t(idx), MPC_results.y(idx), 'g', 'LineWidth', 1.5);
    plot(DeePC_results.t(idx), DeePC_results.y(idx), 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Zoom: First Setpoint Change');
    grid on;
    
    % Zoom in: Second setpoint change (350-450s)
    subplot(3,3,5);
    idx = DMC_results.t >= 350 & DMC_results.t <= 450;
    plot(DMC_results.t(idx), DMC_results.setpoint(idx), 'k--', 'LineWidth', 2); hold on;
    plot(DMC_results.t(idx), DMC_results.y(idx), 'b', 'LineWidth', 1.5);
    plot(MPC_results.t(idx), MPC_results.y(idx), 'g', 'LineWidth', 1.5);
    plot(DeePC_results.t(idx), DeePC_results.y(idx), 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Zoom: Second Setpoint Change');
    grid on;
    
    % Control effort comparison (rate of change)
    subplot(3,3,6);
    du_dmc = [0; diff(DMC_results.u)];
    du_mpc = [0; diff(MPC_results.u)];
    du_deepc = [0; diff(DeePC_results.u)];
    plot(DMC_results.t, abs(du_dmc), 'b', 'LineWidth', 1.5); hold on;
    plot(MPC_results.t, abs(du_mpc), 'g', 'LineWidth', 1.5);
    plot(DeePC_results.t, abs(du_deepc), 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('|du| [%/sample]');
    legend('DMC', 'MPC', 'DeePC', 'Location', 'best');
    title('Control Rate of Change');
    grid on;
    
    % Performance metrics bar chart
    subplot(3,3,7);
    metrics = [DMC_results.MAE, MPC_results.MAE, DeePC_results.MAE; ...
               DMC_results.RMSE, MPC_results.RMSE, DeePC_results.RMSE];
    bar(metrics);
    set(gca, 'XTickLabel', {'MAE', 'RMSE'});
    ylabel('Error [°C]');
    legend('DMC', 'MPC', 'DeePC', 'Location', 'best');
    title('Tracking Error Metrics');
    grid on;
    
    % ISE and IAE comparison
    subplot(3,3,8);
    metrics_int = [DMC_results.ISE, MPC_results.ISE, DeePC_results.ISE; ...
                   DMC_results.IAE, MPC_results.IAE, DeePC_results.IAE];
    bar(metrics_int);
    set(gca, 'XTickLabel', {'ISE', 'IAE'});
    ylabel('Integrated Error');
    legend('DMC', 'MPC', 'DeePC', 'Location', 'best');
    title('Integrated Error Metrics');
    grid on;
    
    % Control effort comparison
    subplot(3,3,9);
    control_efforts = [DMC_results.control_effort, MPC_results.control_effort, ...
                       DeePC_results.control_effort];
    bar(control_efforts);
    set(gca, 'XTickLabel', {'DMC', 'MPC', 'DeePC'});
    ylabel('Total Control Effort');
    title('Control Effort Comparison');
    grid on;
    
    sgtitle('Controller Performance Comparison: DMC vs MPC vs DeePC');
    saveas(gcf, 'controller_comparison.png');
    fprintf('  Saved: controller_comparison.png\n');
    
    %% Statistical analysis
    figure('Position', [100, 100, 1200, 400]);
    
    % Error distribution
    subplot(1,3,1);
    histogram(DMC_results.error, 30, 'Normalization', 'probability', ...
              'FaceAlpha', 0.5, 'DisplayName', 'DMC'); hold on;
    histogram(MPC_results.error, 30, 'Normalization', 'probability', ...
              'FaceAlpha', 0.5, 'DisplayName', 'MPC');
    histogram(DeePC_results.error, 30, 'Normalization', 'probability', ...
              'FaceAlpha', 0.5, 'DisplayName', 'DeePC');
    xlabel('Tracking Error [°C]');
    ylabel('Probability');
    legend('Location', 'best');
    title('Error Distribution');
    grid on;
    
    % Cumulative error
    subplot(1,3,2);
    plot(DMC_results.t, cumsum(abs(DMC_results.error))*DMC_results.t(2), 'b', 'LineWidth', 1.5); hold on;
    plot(MPC_results.t, cumsum(abs(MPC_results.error))*MPC_results.t(2), 'g', 'LineWidth', 1.5);
    plot(DeePC_results.t, cumsum(abs(DeePC_results.error))*DeePC_results.t(2), 'r', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Cumulative Absolute Error');
    legend('DMC', 'MPC', 'DeePC', 'Location', 'best');
    title('Cumulative Tracking Error');
    grid on;
    
    % Settling time analysis (approximate)
    subplot(1,3,3);
    threshold = 0.5;  % 0.5°C error threshold
    settling_dmc = compute_settling_metrics(DMC_results.t, DMC_results.error, threshold);
    settling_mpc = compute_settling_metrics(MPC_results.t, MPC_results.error, threshold);
    settling_deepc = compute_settling_metrics(DeePC_results.t, DeePC_results.error, threshold);
    
    bar([settling_dmc.mean, settling_mpc.mean, settling_deepc.mean]);
    set(gca, 'XTickLabel', {'DMC', 'MPC', 'DeePC'});
    ylabel('Average Settling Time [s]');
    title(sprintf('Settling Time (±%.1f°C threshold)', threshold));
    grid on;
    
    sgtitle('Statistical Analysis of Controller Performance');
    saveas(gcf, 'statistical_analysis.png');
    fprintf('  Saved: statistical_analysis.png\n');
    
    %% Save comparison summary
    comparison_summary = struct();
    comparison_summary.DMC = DMC_results;
    comparison_summary.MPC = MPC_results;
    comparison_summary.DeePC = DeePC_results;
    comparison_summary.best_mae = controllers{best_mae};
    comparison_summary.best_rmse = controllers{best_rmse};
    comparison_summary.best_ise = controllers{best_ise};
    
    save('comparison_summary.mat', 'comparison_summary');
    fprintf('  Saved: comparison_summary.mat\n');
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