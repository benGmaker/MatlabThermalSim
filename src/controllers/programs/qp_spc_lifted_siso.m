function qp = qp_spc_lifted_siso(F, Phi, P, Qw, Rw, opts)
%QP_SPC_LIFTED_SISO Condensed SISO SPC/MPC QP for lifted predictor with Δu handling.
%
% Predictor (deviation form):
%   Y = F*xk + Phi*U
% where U = [u0 ... u_{P-1}]'
%
% Cost:
%   sum ||y - r||_Q^2 + ||u||_R^2 + ||Δu||_S^2   (optional)
%
% Constraints:
%   umin <= u_k <= umax
%   ymin <= y_k <= ymax (optional)
%   dumin <= Δu_k <= dumax (optional), with Δu0 = u0 - u_prev
%

    % ---- dimensions ----
    if size(F,1) ~= P || size(Phi,1) ~= P || size(Phi,2) ~= P
        error('qp_spc_lifted_siso: F must be (P x nx) and Phi must be (P x P).');
    end
    nx = size(F,2);

    % ---- weights ----
    Q = Qw * eye(P);
    R = Rw * eye(P);

    % Base Hessian for tracking + input magnitude
    H_base = 2*(Phi' * Q * Phi + R);
    H_base = (H_base + H_base')/2;

    % ---- Δu difference operator ----
    % ΔU = D*U - d0, where d0(1)=u_prev, others 0
    D = eye(P);
    for k = 2:P
        D(k, k-1) = -1;
    end

    if opts.constraints.enable_du_penalty
        S = opts.constraints.du_weight * eye(P);
        H_du = 2*(D' * S * D);
    else
        S = zeros(P);
        H_du = zeros(P);
    end

    qp = struct();
    qp.nx = nx;
    qp.P = P;
    qp.Qw = Qw; qp.Rw = Rw;
    qp.Q = Q;   qp.R = R;
    qp.F = F;
    qp.Phi = Phi;

    qp.enable_y_constraints   = logical(opts.constraints.enable_y_constraints);
    qp.enable_du_penalty      = logical(opts.constraints.enable_du_penalty);
    qp.enable_du_constraints  = logical(opts.constraints.enable_du_constraints);

    qp.du_weight = opts.constraints.du_weight;
    qp.du_min = opts.constraints.du_min;
    qp.du_max = opts.constraints.du_max;

    qp.solve = @solve_step;

    function [u0, info] = solve_step(xk, r_dev, u_prev_dev, umin_dev, umax_dev, ymin_dev, ymax_dev)
        % ---- reference shaping ----
        r = r_dev(:);
        if numel(r) < P
            r(end+1:P,1) = r(end);
        elseif numel(r) > P
            r = r(1:P);
        end

        % ---- affine prediction due to state ----
        y_aff = F*xk;

        % Base gradient from tracking term
        f = 2*(Phi' * Q * (y_aff - r));

        % Δu affine term d0 so ΔU = D*U - d0
        d0 = zeros(P,1);
        d0(1) = u_prev_dev;

        % Add Δu penalty: (D U - d0)' S (D U - d0)
        if qp.enable_du_penalty
            % Gradient contribution: 2*D'*S*(D*U - d0) => adds -2*D'*S*d0 to f
            f = f + 2*(D' * S * (-d0));
        end

        % Final Hessian
        H = H_base + H_du;
        H = (H + H')/2;
        H = H + 1e-9*eye(P);

        % ---- bounds on U ----
        lb = umin_dev * ones(P,1);
        ub = umax_dev * ones(P,1);

        % ---- inequality constraints ----
        Aineq = [];
        bineq = [];

        % Output constraints: ymin <= y_aff + Phi U <= ymax
        if qp.enable_y_constraints
            Aineq = [Aineq;  Phi; -Phi];
            bineq = [bineq; (ymax_dev*ones(P,1) - y_aff);
                           -(ymin_dev*ones(P,1) - y_aff)];
        end

        % Δu constraints: du_min <= D*U - d0 <= du_max
        if qp.enable_du_constraints
            du_min = qp.du_min;
            du_max = qp.du_max;

            Aineq = [Aineq;  D; -D];
            bineq = [bineq; (du_max*ones(P,1) + d0);
                           -(du_min*ones(P,1) + d0)];
        end

        options = optimoptions('quadprog', ...
            'Display', 'off', ...
            'MaxIterations', 200, ...
            'ConstraintTolerance', 1e-7, ...
            'OptimalityTolerance', 1e-7);

        info = struct('status', -1, 'exitflag', [], 'U', zeros(P,1));

        try
            [U_opt, ~, exitflag, output] = quadprog(H, f, Aineq, bineq, [], [], lb, ub, [], options);
            info.exitflag = exitflag;
            info.output = output;

            if exitflag > 0 && ~isempty(U_opt)
                info.status = 0;
                info.U = U_opt(:);
                u0 = U_opt(1);
            else
                % safe fallback: hold last move (rate-friendly)
                u0 = min(max(u_prev_dev, umin_dev), umax_dev);
            end
        catch ME
            info.status = -2;
            info.exitflag = NaN;
            info.error_identifier = ME.identifier;
            info.error_message = ME.message;
            info.error_stack = ME.stack;
            rethrow(ME);
        end

    end
end