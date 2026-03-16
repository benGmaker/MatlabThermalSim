function [Up, Yp, Uf, Yf] = spc_hankel_past_future_siso(u, y, M, P)
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



    u = u(:); y = y(:);

    L  = M + P;
    Hu = spc_build_hankel_siso(u, L);
    Hy = spc_build_hankel_siso(y, L);

    check_pe_condition(Hu);

    Up = Hu(1:M, :);
    Uf = Hu(M+1:M+P, :);

    Yp = Hy(1:M, :);
    Yf = Hy(M+1:M+P, :);
end