function qp = qp_mpc_siso(sys_ss, P, M, Qw, Rw, opts)
%QP_MPC_SISO Build and solve a condensed SISO MPC QP using quadprog.
%
% Primary decision variable: dU = [du0 ... du_{M-1}]'
% Prediction uses move blocking after M.
%
% Optional offset-free tracking:
%   opts.enable_integrator (default false)
%
% Call:
%   qp = qp_mpc_siso(sys_ss, P, M, Qw, Rw);
%   [du0, info] = qp.solve(xk_aug, r_dev, u_prev_dev, umin_dev, umax_dev, du_max);

    if nargin < 6, opts = struct(); end
    if ~isfield(opts,'enable_integrator'), opts.enable_integrator = false; end

    sys_ss = ss(sys_ss);
    if ~isdt(sys_ss)
        error('qp_mpc_siso: sys_ss must be discrete-time.');
    end

    [A,B,C,D] = ssdata(sys_ss);

    % SISO check
    if size(B,2) ~= 1 || size(C,1) ~= 1 || size(D,2) ~= 1
        error('qp_mpc_siso: only SISO models are supported.');
    end
    enableI = opts.enable_integrator;

    if enableI
        % y_tilde = [C 1] * [x; xi] + D u
        A_aug = [A zeros(size(A,1),1);
                -C 1];
        B_aug = [B;
                -D];
        C_aug = [C 1];
        D_aug = D;
    else
        A_aug = A; B_aug = B; C_aug = C; D_aug = D;
    end

    nx = size(A_aug,1);

    qp = struct();
    qp.A = A_aug; qp.B = B_aug; qp.C = C_aug; qp.D = D_aug;
    qp.nx = nx;
    qp.P = P; qp.M = M;
    qp.Qw = Qw; qp.Rw = Rw;
    qp.enable_integrator = enableI;

    % Build prediction matrices for Y = F*xk + Phi*U (U length P)
    F = zeros(P, nx);
    Phi = zeros(P, P);

    for i = 1:P
        Ai = A_aug^i;
        F(i,:) = (C_aug * Ai);

        for j = 1:i
            Aij = A_aug^(i-j);
            Phi(i,j) = C_aug * Aij * B_aug;
        end
        Phi(i,i) = Phi(i,i) + D_aug;
    end

    % Map dU (M) -> U (P) with move blocking
    S = zeros(P, M);
    for i = 1:P
        for j = 1:M
            if j <= min(i, M)
                S(i,j) = 1;
            end
        end
    end
    oneP = ones(P,1);

    Gdu = Phi * S;  % P x M

    Q = Qw * eye(P);
    R = Rw * eye(M);

    H = 2*(Gdu' * Q * Gdu + R);
    H = (H + H')/2;

    qp.F = F;
    qp.Phi = Phi;
    qp.S = S;
    qp.oneP = oneP;
    qp.Gdu = Gdu;
    qp.H = H;
    qp.Q = Q;
    qp.R = R;

    qp.solve = @solve_step;

    function [du0, info] = solve_step(xk, r_dev, u_prev_dev, umin_dev, umax_dev, du_max)
        r = r_dev(:);
        if numel(r) < P
            r(end+1:P,1) = r(end);
        elseif numel(r) > P
            r = r(1:P);
        end

        y_aff = F*xk + Phi*(oneP*u_prev_dev);

        % Minimize ||(y_aff + Gdu*dU) - r||_Q^2 + ||dU||_R^2
        f = 2*(Gdu' * Q * (y_aff - r));

        % Input bounds on predicted U:
        % U = oneP*u_prev + S*dU
        A_u = [ S; -S ];
        b_u = [ (umax_dev - u_prev_dev)*ones(P,1);
               -(umin_dev - u_prev_dev)*ones(P,1) ];

        lb = -du_max * ones(M,1);
        ub =  du_max * ones(M,1);

        options = optimoptions('quadprog', ...
            'Display', 'off', ...
            'MaxIterations', 200, ...
            'ConstraintTolerance', 1e-7, ...
            'OptimalityTolerance', 1e-7);

        info = struct('status', -1, 'exitflag', [], 'du', zeros(M,1));

        try
            [dU_opt, ~, exitflag, output] = quadprog(H, f, A_u, b_u, [], [], lb, ub, [], options);
            info.exitflag = exitflag;
            info.output = output;
        
            if exitflag > 0 && ~isempty(dU_opt)
                info.status = 0;
                info.du = dU_opt(:);
                du0 = dU_opt(1);
            else
                du0 = 0;
            end
        catch ME
            info.status = -2;
            info.exitflag = NaN;
            info.error_identifier = ME.identifier;
            info.error_message = ME.message;
            info.error_stack = ME.stack;
            rethrow(ME);  % temporarily rethrow so you can see the real error
        end
    end
end