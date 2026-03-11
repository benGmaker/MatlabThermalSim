function [Up, Yp, Uf, Yf] = spc_hankel_past_future_siso(u, y, i)
%SPC_HANKEL_PAST_FUTURE_SISO Build past/future Hankel blocks for SISO.
% Up,Yp,Uf,Yf are i x j with j = T-2i+1.

    u = u(:); y = y(:);
    Hu = spc_build_hankel_siso(u, 2*i);
    Hy = spc_build_hankel_siso(y, 2*i);

    Up = Hu(1:i, :);
    Uf = Hu(i+1:2*i, :);

    Yp = Hy(1:i, :);
    Yf = Hy(i+1:2*i, :);
end