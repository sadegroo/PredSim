function [eps_rel,eps_abs] = getMeshError(Qs,Qs_col,Qdots,h,C,d)
% --------------------------------------------------------------------------
% getMeshError
%   Estimates the local discretization error of each mesh interval, following
%   Betts (2010), Practical Methods for Optimal Control and Estimation Using
%   Nonlinear Programming, section 4.7.2.
%
%   The estimate uses that the residual of the Qs collocation equation is a
%   polynomial with the collocation points as its roots, so it is fully
%   determined by its value at the start of the interval.
%
% INPUT:
%   - Qs, Qdots -
%   * unscaled coordinate values and velocities at the mesh points (N+1 x nq)
%
%   - Qs_col -
%   * unscaled coordinate values at the collocation points (d*N x nq)
%
%   - h -
%   * duration of each mesh interval (1 x N) [s]
%
%   - C -
%   * coefficients of the collocation equation, see CollocationScheme
%
%   - d -
%   * degree of the interpolating polynomial
%
% OUTPUT:
%   - eps_rel -
%   * relative local error of each mesh interval (1 x N)
%
%   - eps_abs -
%   * absolute local error per coordinate (N x nq)
%
% Original author: Sander De Groof
% Original date: 11/August/2026
%
% --------------------------------------------------------------------------
% This file is part of PredSim.
%
% PredSim: A Framework for Rapid Predictive Simulations of Locomotion
% Copyright (c) 2026 KU Leuven
%
% PredSim is free software: you can redistribute it and/or modify it under
% the terms of the GNU Affero General Public License as published by the
% Free Software Foundation, either version 3 of the License, or (at your
% option) any later version.
%
% PredSim is distributed in the hope that it will be useful, but WITHOUT
% ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
% FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Affero General Public
% License for more details.
%
% You should have received a copy of the GNU Affero General Public License
% along with PredSim. If not, see <https://www.gnu.org/licenses/>.
% --------------------------------------------------------------------------

assert(d==3, ['getMeshError: c_int is the residual shape constant for ' ...
    '3-stage Radau IIA and must be recomputed from tau_root for any other degree.']);

N = numel(h);
c_int = 0.235151; % integral of the residual shape over an interval, d = 3 radau

eps_abs = zeros(N,size(Qs,2));
for k=1:N
    Qskj = [Qs(k,:); Qs_col((k-1)*d+1:k*d,:)];  % d+1 x nq
    r0 = (C(:,1)'*Qskj)/h(k) - Qdots(k,:);      % residual at the start of the interval
    eps_abs(k,:) = h(k)*abs(r0)*c_int;
end

w = max(max(abs(Qs(1:N,:)),abs(Qdots(1:N,:))),[],1);
eps_rel = max(eps_abs./(w+1),[],2)';

end
