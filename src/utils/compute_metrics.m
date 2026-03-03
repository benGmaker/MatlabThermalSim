function m = compute_metrics(error, u, dt)
%COMPUTE_METRICS Common performance metrics

    m = struct();
    m.MAE  = mean(abs(error));
    m.RMSE = sqrt(mean(error.^2));
    m.ISE  = sum(error.^2) * dt;
    m.IAE  = sum(abs(error)) * dt;

    du = [0; diff(u)];
    m.control_effort = sum(abs(du));
end