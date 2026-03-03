function T_setpoint_abs = build_setpoint_profile(config)

    t_sim = config.simulation.t_sim;
    dt = config.simulation.dt;
    t = 0:dt:t_sim;
    n_steps = length(t);

    T_setpoint_abs = ones(n_steps, 1) * config.setpoint.values(1);

    for i = 1:length(config.setpoint.times)
        idx = round(config.setpoint.times(i) / dt) + 1;
        if idx <= n_steps
            T_setpoint_abs(idx:end) = config.setpoint.values(i);
        end
    end
end