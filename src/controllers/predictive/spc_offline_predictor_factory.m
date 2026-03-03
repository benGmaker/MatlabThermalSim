function [predictor_step, predictor_init, meta] = spc_offline_predictor_factory(config)
%SPC_OFFLINE_PREDICTOR_FACTORY
% Identifies an SS model from multisine (same as spc_policy_factory),
% then uses it as a one-step-ahead predictor.

    if nargin < 1, config = config_simulation(); end

    load('results/multisine_response_data.mat', 'multisine_data');

    u_abs = multisine_data.Q(:);
    y_abs = multisine_data.T(:);
    dt = multisine_data.params.dt;

    % center like spc_policy_factory
    u_mean = mean(u_abs);
    y_mean = mean(y_abs);
    u_id = u_abs - u_mean;
    y_id = y_abs - y_mean;

    z = iddata(y_id, u_id, dt);
    nx = config.SPC.ident.nx;

    switch lower(config.SPC.ident.method)
        case 'n4sid'
            opt = n4sidOptions('Focus', config.SPC.ident.focus);
            sys_id = n4sid(z, nx, opt);
        otherwise
            error('Unsupported SPC ident method: %s', config.SPC.ident.method);
    end

    if ~isdt(sys_id)
        sys_id = c2d(sys_id, dt, 'zoh');
    end
    sys_ss = ss(sys_id);
    [A,B,C,D] = ssdata(sys_ss);

    L_gain = 0.5;

    predictor_init = @init;
    predictor_step = @step;

    meta = struct();
    meta.dt = dt;
    meta.sys_ss = sys_ss;
    meta.nx = nx;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.L_gain = L_gain;

    function pred = init()
        pred = struct();
        pred.A = A; pred.B = B; pred.C = C; pred.D = D;
        pred.x = zeros(size(A,1),1);
        pred.u_mean = u_mean;
        pred.y_mean = y_mean;
        pred.L_gain = L_gain;
    end

    function [y_hat_next_abs, pred] = step(k, y_k_abs, u_k_abs, pred, config) %#ok<INUSD>
        y_k = y_k_abs - pred.y_mean;
        u_k = u_k_abs - pred.u_mean;

        y_hat_k = pred.C*pred.x + pred.D*u_k;
        pred.x = pred.x + pred.L_gain * (pred.C') * (y_k - y_hat_k);

        x_next = pred.A*pred.x + pred.B*u_k;
        y_hat_next = pred.C*x_next + pred.D*u_k;

        pred.x = x_next;
        y_hat_next_abs = y_hat_next + pred.y_mean;
    end
end