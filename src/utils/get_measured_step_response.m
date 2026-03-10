% --- reuse your existing helpers (can move to common/ if you want)
function S = get_measured_step_response(N, dt, u_mean, y_mean)
    data = load('results/data/step_response_data.mat');
    step_data = data.step_data;

    if isfield(step_data, 'step_idx')
        step_idx = step_data.step_idx;
    else
        step_idx = find(step_data.Q > mean(step_data.Q), 1, 'first');
    end

    if isfield(step_data, 'step_amplitude')
        step_magnitude = step_data.step_amplitude;
    else
        u_before = mean(step_data.Q(1:max(1,step_idx-1)));
        u_after = mean(step_data.Q(step_idx:min(step_idx+10, end)));
        step_magnitude = u_after - u_before;
    end

    y_before = mean(step_data.T(1:max(1,step_idx-1)));
    y_response = step_data.T(step_idx:end) - y_before;
    y_unit_step = y_response / step_magnitude;

    if length(y_unit_step) >= N
        S = y_unit_step(1:N);
    else
        S = [y_unit_step; y_unit_step(end) * ones(N - length(y_unit_step), 1)];
    end
    S = S(:);
end
