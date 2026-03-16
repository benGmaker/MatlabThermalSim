function qp = qp_mpc_siso(sys_ss, P, Qw, Rw, opts)
%QP_MPC_SISO Build and solve condensed SISO MPC QP using quadprog.
%
% Theory-matching nominal MPC:
%   Decision variable: U = [u0 ... u_{P-1}]'
%   Prediction: Y = F*xk + Phi*U
%   Cost: sum ||y_k - r_k||_Q^2 + ||u_k||_R^2
%
% No offset-free augmentation (per report request).

    if nargin < 5, opts = struct(); end
    if ~isfield(opts,'enable_y_constraints'), opts.enable_y_constraints = true; end

    sys_ss = ss(sys_ss);
    if ~isdt(sys_ss)
        error('qp_mpc_siso: sys_ss must be discrete-time.');
    end

    [A,B,C,D] = ssdata(sys_ss);

    % SISO check
    if size(B,2) ~= 1 || size(C,1) ~= 1 || size(D,2) ~= 1
        error('qp_mpc_siso: only SISO models are supported.');
    end

    nx = size(A,1);

    qp = struct();
    qp.A = A; qp.B = B; qp.C = C; qp.D = D;
    qp.nx = nx;
    qp.P = P;
    qp.Qw = Qw; qp.Rw = Rw;
    qp.enable_y_constraints = logical(opts.enable_y_constraints);

    % Build prediction matrices for Y = F*xk + Phi*U (U length P)
    F = zeros(P, nx);
    Phi = zeros(P, P);

    for i = 1:P
        Ai = A^i;
        F(i,:) = (C * Ai);

        for j = 1:i
            Aij = A^(i-j);
            Phi(i,j) = C * Aij * B;
        end
        Phi(i,i) = Phi(i,i) + D;
    end

    Q = Qw * eye(P);
    R = Rw * eye(P);

    H = 2*(Phi' * Q * Phi + R);
    H = (H + H')/2;

    qp.F = F;
    qp.Phi = Phi;
    qp.H = H;
    qp.Q = Q;
    qp.R = R;

    qp.solve = @solve_step;

    function [u0, info] = solve_step(xk, r_dev, umin_dev, umax_dev, ymin_dev, ymax_dev)
        r = r_dev(:);
        if numel(r) < P
            r(end+1:P,1) = r(end);
        elseif numel(r) > P
            r = r(1:P);
        end

        % Affine output due to state
        y_aff = F*xk;

        % Minimize ||(y_aff + Phi*U) - r||_Q^2 + ||U||_R^2
        f = 2*(Phi' * Q * (y_aff - r));

        % Input bounds on predicted U
        lb = umin_dev * ones(P,1);
        ub = umax_dev * ones(P,1);

        % Optional output constraints: ymin <= y_aff + Phi U <= ymax
        if qp.enable_y_constraints
            Aineq = [ Phi; -Phi ];
            bineq = [ (ymax_dev*ones(P,1) - y_aff);
                     -(ymin_dev*ones(P,1) - y_aff) ];
        else
            Aineq = [];
            bineq = [];
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
                u0 = min(max(0, umin_dev), umax_dev); % safe fallback
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