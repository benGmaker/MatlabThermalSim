function compare_predictors(config)
%COMPARE_PREDICTORS
% Postprocess and compare offline predictors.
% Loads results/OfflinePred_all_predictors.mat and produces:
%  - CLI table of metrics
%  - plots of y_meas vs y_hat
%  - error distribution plots
% Saves PNGs in results/.

    if nargin < 1
        config = config_simulation();
    end

    load('results/OfflinePred_all_predictors.mat', 'AllPred_results');

    items = AllPred_results.items;
    n = numel(items);
    if n == 0
        error('No predictor items found in OfflinePred_all_predictors.mat');
    end

    % Colors (reuse config if available; otherwise fallback)
    colors = struct();
    if isfield(config, 'plotting') && isfield(config.plotting, 'colors')
        colors = config.plotting.colors;
    end
    if ~isfield(colors, 'MPC'),   colors.MPC   = [0.00, 0.45, 0.74]; end
    if ~isfield(colors, 'DMC'),   colors.DMC   = [0.85, 0.33, 0.10]; end
    if ~isfield(colors, 'DeePC'), colors.DeePC = [0.47, 0.67, 0.19]; end
    if ~isfield(colors, 'SPC'),   colors.SPC   = [0.49, 0.18, 0.56]; end

    % Map predictor names to colors consistently
    name_to_color = containers.Map();
    name_to_color('MPC_sys_ss')   = colors.MPC;
    name_to_color('DMC_step')     = colors.DMC;
    name_to_color('DeePC_hankel') = colors.DeePC;
    name_to_color('SPC_n4sid')    = colors.SPC;

    % ---- Print comparison table
    fprintf('\n========== Offline Predictor Comparison ==========\n');
    fprintf('%-18s %-12s %-12s %-12s\n', 'Predictor', 'MAE [°C]', 'RMSE [°C]', 'Fit [%]');
    fprintf('%-18s %-12s %-12s %-12s\n', repmat('-',1,18), repmat('-',1,12), repmat('-',1,12), repmat('-',1,12));

    maes  = nan(n,1);
    rmses = nan(n,1);
    fits  = nan(n,1);
    names = strings(n,1);

    for i = 1:n
        names(i) = string(items(i).name);
        maes(i)  = items(i).results.metrics.MAE;
        rmses(i) = items(i).results.metrics.RMSE;
        fits(i)  = items(i).results.metrics.fit_percent;

        fprintf('%-18s %-12.4f %-12.4f %-12.2f\n', ...
            items(i).name, maes(i), rmses(i), fits(i));
    end
    fprintf('===============================================\n');

    [~, best_rmse_idx] = min(rmses);
    fprintf('Best RMSE: %s (%.4f °C)\n', items(best_rmse_idx).name, rmses(best_rmse_idx));

    % Use first item for shared signals
    t = items(1).results.t;
    y_meas = items(1).results.y_meas;

    % ---- Plot y_meas vs y_hat for all predictors
    figure('Position', [100, 100, 1600, 700]);

    subplot(2,2,[1 2]);
    plot(t, y_meas, 'k', 'LineWidth', 1.8, 'DisplayName', 'Measured'); hold on;
    for i = 1:n
        y_hat = items(i).results.y_hat;
        c = pick_color(name_to_color, items(i).name);
        plot(t, y_hat, 'Color', c, 'LineWidth', 1.2, 'DisplayName', items(i).name);
    end
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    title('Offline 1-step-ahead prediction: y_{meas} vs y_{hat}');
    legend('Location','best');
    grid on;
    ymin = min(y_meas);
    ymax = max(y_meas);
    pad = 0.05 * (ymax - ymin);
    if pad == 0, pad = 1; end  % degenerate case
    ylim([ymin - pad, ymax + pad]);

    % ---- Error time series
    subplot(2,2,3);
    
    all_err = [];  % collect errors across predictors for common y-limits
    
    for i = 1:n
        e = items(i).results.error;
        c = pick_color(name_to_color, items(i).name);
        plot(t, e, 'Color', c, 'LineWidth', 1.0, 'DisplayName', items(i).name); hold on;
    
        % collect finite values (skip first sample)
        if numel(e) >= 2
            all_err = [all_err; e(2:end)]; %#ok<AGROW>
        else
            all_err = [all_err; e(:)]; %#ok<AGROW>
        end
    end
    
    yline(0,'k-');
    xlabel('Time [s]'); ylabel('Prediction error [°C]');
    title('Prediction error e = y_{meas} - y_{hat}');
    legend('Location','best');
    grid on;
    
    % Robust y-limits (symmetric around 0) to prevent blow-ups
    all_err = all_err(isfinite(all_err));
    if ~isempty(all_err)
        % Use absolute error percentile and enforce symmetric limits
        abs_lim = prctile(abs(all_err), 80);   % try 90 or 95; 99 is usually too wide
        if abs_lim == 0
            abs_lim = max(abs(all_err));
            if abs_lim == 0, abs_lim = 0.5; end
        end
        pad = 0.10 * abs_lim;
        ylim([-abs_lim - pad, abs_lim + pad]);
    end

    % ---- Metrics bar chart (MAE & RMSE)
    subplot(2,2,4);
    metric_mat = [maes, rmses]; % n x 2
    % b = bar(metric_mat);
    grid on;
    set(gca, 'XTickLabel', cellstr(names));
    ylabel('Error [°C]');
    title('MAE and RMSE per predictor');
    legend({'MAE','RMSE'}, 'Location','best');
    xtickangle(20);

    sgtitle('Offline Predictor Comparison');
    saveas(gcf, 'results/predictor_comparison.png');
    fprintf('Saved: results/predictor_comparison.png\n');

    % ---- Error distribution plot
    figure('Position', [100, 100, 1600, 500]);
    for i = 1:n
        e = items(i).results.error;
        e = e(2:end); % skip first
        c = pick_color(name_to_color, items(i).name);

        histogram(e, 50, 'Normalization','probability', ...
            'FaceColor', c, 'FaceAlpha', 0.35, 'EdgeAlpha', 0.2, ...
            'DisplayName', items(i).name); hold on;
    end
    xlabel('Prediction error [°C]');
    ylabel('Probability');
    title('Prediction error distributions (offline 1-step)');
    legend('Location','best');
    grid on;
    saveas(gcf, 'results/predictor_error_distributions.png');
    fprintf('Saved: results/predictor_error_distributions.png\n');

    % ---- Save summary mat
    summary = struct();
    summary.created_at = char(datetime('now'));
    summary.names = cellstr(names);
    summary.mae = maes;
    summary.rmse = rmses;
    summary.fit_percent = fits;
    summary.best_rmse_name = items(best_rmse_idx).name;
    summary.best_rmse_value = rmses(best_rmse_idx);
    save('results/predictor_comparison_summary.mat', 'summary');
    fprintf('Saved: results/predictor_comparison_summary.mat\n');
end

function c = pick_color(name_to_color, predictor_name)
    if isKey(name_to_color, predictor_name)
        c = name_to_color(predictor_name);
    else
        c = [0.2 0.2 0.2];
    end
end