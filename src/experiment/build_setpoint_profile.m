function T_setpoint = build_setpoint_profile(config)
% BUILD_SETPOINT_PROFILE Create setpoint trajectory from configuration
%
% Inputs:
%   config - Configuration struct from config_simulation()
%
% Outputs:
%   T_setpoint - Column vector of setpoint values [°C]

    t = 0:config.simulation.dt:config.simulation.t_sim;
    n_steps = length(t);
    T_setpoint = zeros(n_steps, 1);
    
    % Build piecewise constant setpoint
    for i = 1:length(config.setpoint.times)
        if i < length(config.setpoint.times)
            % Not the last segment
            idx_start = find(t >= config.setpoint.times(i), 1, 'first');
            idx_end = find(t < config.setpoint.times(i+1), 1, 'last');
            T_setpoint(idx_start:idx_end) = config.setpoint.values(i);
        else
            % Last segment - goes to end
            idx_start = find(t >= config.setpoint.times(i), 1, 'first');
            T_setpoint(idx_start:end) = config.setpoint.values(i);
        end
    end
end