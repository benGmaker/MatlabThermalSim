function pe = check_pe_condition(H, opts)
%CHECK_PE_CONDITION Persistent excitation (PE) / full-row-rank diagnostic for a Hankel matrix.
%
% Usage:
%   pe = spc_check_pe(H)
%   pe = spc_check_pe(H, opts)
%
% Inputs:
%   H    : (L x j) matrix (typically a Hankel matrix built from an input signal)
%   opts : (optional) struct with fields:
%          - name       (string) label for printing, default 'H'
%          - do_print   (logical) default true
%          - rank_tol   (scalar) tolerance for rank(H, tol); default []
%          - sv_rel_tol (scalar) warning threshold for sigma_min/sigma_max; default 1e-8
%          - cond_warn  (scalar) warning threshold for condition number; default 1e10
%
% Output struct pe:
%   pe.L               number of rows
%   pe.j               number of columns
%   pe.rank            numerical rank
%   pe.full_row_rank   true if rank == L and j >= L
%   pe.sigma_max
%   pe.sigma_min
%   pe.sv_ratio        sigma_min/sigma_max
%   pe.cond_est        sigma_max/sigma_min (Inf if sigma_min~0)
%   pe.warnings        cellstr of warning messages

    if nargin < 2 || isempty(opts), opts = struct(); end

    if ~isfield(opts,'name'),       opts.name = 'H'; end
    if ~isfield(opts,'do_print'),   opts.do_print = true; end
    if ~isfield(opts,'rank_tol'),   opts.rank_tol = []; end % if empty, MATLAB chooses
    if ~isfield(opts,'sv_rel_tol'), opts.sv_rel_tol = 1e-8; end
    if ~isfield(opts,'cond_warn'),  opts.cond_warn  = 1e10; end

    [L, j] = size(H);

    % SVD metrics
    s = svd(H, 'econ');
    if isempty(s)
        sigma_max = 0;
        sigma_min = 0;
    else
        sigma_max = s(1);
        sigma_min = s(end);
    end

    sv_ratio = sigma_min / max(sigma_max, eps);
    cond_est = sigma_max / max(sigma_min, eps);

    % Numerical rank
    if isempty(opts.rank_tol)
        r = rank(H);
    else
        r = rank(H, opts.rank_tol);
    end

    full_row_rank = (j >= L) && (r == L);

    warnings = {};

    if j < L
        warnings{end+1} = sprintf('Not enough columns for full row rank: j=%d < L=%d.', j, L);
    end
    if r < L
        warnings{end+1} = sprintf('Rank-deficient: rank(H)=%d < L=%d (not PE of order L).', r, L);
    end
    if sv_ratio < opts.sv_rel_tol
        warnings{end+1} = sprintf('Near loss of excitation: sigma_min/sigma_max=%.3e < %.3e.', sv_ratio, opts.sv_rel_tol);
    end
    if cond_est > opts.cond_warn
        warnings{end+1} = sprintf('Ill-conditioned: cond~%.3e > %.3e.', cond_est, opts.cond_warn);
    end

    pe = struct();
    pe.L = L;
    pe.j = j;
    pe.rank = r;
    pe.full_row_rank = full_row_rank;
    pe.sigma_max = sigma_max;
    pe.sigma_min = sigma_min;
    pe.sv_ratio = sv_ratio;
    pe.cond_est = cond_est;
    pe.warnings = warnings;

    if opts.do_print
        fprintf('[SPC][PE] %s: size=%dx%d\n', opts.name, L, j);
        fprintf('[SPC][PE] rank=%d (target full row rank=%d), full_row_rank=%d\n', r, L, full_row_rank);
        fprintf('[SPC][PE] sigma_max=%.3e, sigma_min=%.3e, sigma_min/sigma_max=%.3e, cond~%.3e\n', ...
            sigma_max, sigma_min, sv_ratio, cond_est);

        for k = 1:numel(warnings)
            fprintf('[SPC][PE][WARN] %s\n', warnings{k});
        end
    end
end