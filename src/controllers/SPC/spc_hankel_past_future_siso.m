function [Up, Yp, Uf, Yf] = spc_hankel_past_future_siso(u, y, M, P, do_print, pe_opts)
%SPC_HANKEL_PAST_FUTURE_SISO Build past/future Hankel blocks for SISO.
%
% Past horizon:  M  (history length)
% Future horizon: P (equals MPC horizon)
%
% Up,Yp are M x j
% Uf,Yf are P x j
% where j = T - (M+P) + 1
%
% Optional PE diagnostic (input persistent excitation):
%   Checks if Hu (Hankel of input, depth L=M+P) is (numerically) full row rank.

    if nargin < 5 || isempty(do_print), do_print = true; end % settings need to be implemented properly
    if nargin < 6 || isempty(pe_opts), pe_opts = struct(); end

    % Options / thresholds
    if ~isfield(pe_opts, 'sv_rel_tol'),  pe_opts.sv_rel_tol  = 1e-8;  end % sigma_min/sigma_max warning
    if ~isfield(pe_opts, 'rank_tol'),    pe_opts.rank_tol    = [];     end % if empty, MATLAB chooses
    if ~isfield(pe_opts, 'cond_warn'),   pe_opts.cond_warn   = 1e10;   end % condition number warning
    if ~isfield(pe_opts, 'name'),        pe_opts.name        = 'u';    end % label for printing

    u = u(:); y = y(:);

    L  = M + P;
    Hu = spc_build_hankel_siso(u, L);
    Hy = spc_build_hankel_siso(y, L);

    % ---- Persistent excitation diagnostic (input) ----
    if do_print
        % SVD-based metrics
        s = svd(Hu, 'econ');
        if isempty(s)
            sigma_max = 0; sigma_min = 0; rel = 0; c = Inf;
        else
            sigma_max = s(1);
            sigma_min = s(end);
            rel = sigma_min / max(sigma_max, eps);
            c = sigma_max / max(sigma_min, eps);
        end

        % Numerical rank (full row rank desired: rank(Hu) == L)
        if isempty(pe_opts.rank_tol)
            r = rank(Hu);
        else
            r = rank(Hu, pe_opts.rank_tol);
        end

        j = size(Hu,2);

        fprintf('[SPC][PE] Hankel(%s): L=M+P=%d, cols j=%d\n', pe_opts.name, L, j);
        fprintf('[SPC][PE] rank(Hu)=%d (target full row rank = %d)\n', r, L);
        fprintf('[SPC][PE] sigma_max=%.3e, sigma_min=%.3e, sigma_min/sigma_max=%.3e, cond~%.3e\n', ...
            sigma_max, sigma_min, rel, c);

        % Warnings
        if j < L
            fprintf('[SPC][PE][WARN] Not enough columns for full row rank: j=%d < L=%d.\n', j, L);
        end
        if r < L
            fprintf('[SPC][PE][WARN] Input Hankel is rank-deficient (NOT persistently exciting of order %d).\n', L);
        else
            fprintf('[SPC][PE] Input Hankel is full row rank -> PE of order %d (numerically).\n', L);
        end
        if rel < pe_opts.sv_rel_tol
            fprintf('[SPC][PE][WARN] Near loss of excitation: sigma_min/sigma_max=%.3e < %.3e.\n', rel, pe_opts.sv_rel_tol);
        end
        if c > pe_opts.cond_warn
            fprintf('[SPC][PE][WARN] Ill-conditioned Hankel: cond~%.3e > %.3e (pinv may amplify noise).\n', c, pe_opts.cond_warn);
        end
    end

    Up = Hu(1:M, :);
    Uf = Hu(M+1:M+P, :);

    Yp = Hy(1:M, :);
    Yf = Hy(M+1:M+P, :);
end