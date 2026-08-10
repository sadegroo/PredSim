function dtau = getMeshRefinedAtWindows(N,windows)
% --------------------------------------------------------------------------
% getMeshRefinedAtWindows
%   Builds a non-uniform normalised mesh vector so a user does not have to
%   hand write a long vector for S.solver.mesh_intervals. Starts from N
%   uniform intervals; each interval whose midpoint falls inside a window
%   is split into 'factor' equal parts. Overlapping windows are resolved by
%   taking the largest applicable factor for that interval. The result is
%   normalised to sum to 1.
%
% INPUT:
%   - N -
%   * number of uniform mesh intervals before refinement, positive integer
%   scalar
%
%   - windows -
%   * n x 3 array, each row [start, stop, factor] with start and stop in
%   [0,1] as fraction of the simulated motion and factor >= 1 (rounded to
%   the nearest integer). Empty windows returns the uniform mesh
%   ones(1,N)/N. On a HalfGaitCycle simulation the simulated motion is the
%   half cycle, so a window placed on heel strike refines both heel
%   strikes of the reconstructed full cycle.
%
% OUTPUT:
%   - dtau -
%   * 1 x M row of relative interval durations, strictly positive and
%   summing to 1, with M >= N
%
% Original author: Sander De Groof
% Original date: 10/August/2026
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

% Validate N.
if ~isscalar(N) || N < 1 || round(N) ~= N
    error('getMeshRefinedAtWindows:invalidN','N must be a positive integer scalar.');
end

% Validate windows.
if isempty(windows)
    dtau = ones(1,N)/N;
    return;
end
if size(windows,2) ~= 3
    error('getMeshRefinedAtWindows:invalidWindows','windows must be n x 3, each row [start, stop, factor].');
end
starts = windows(:,1);
stops = windows(:,2);
if any(starts < 0) || any(stops > 1) || any(starts >= stops)
    error('getMeshRefinedAtWindows:invalidBounds','each window needs 0 <= start < stop <= 1.');
end
if any(windows(:,3) < 1)
    error('getMeshRefinedAtWindows:invalidFactor','factor must be >= 1.');
end
factors = round(windows(:,3)); % fractional splits are meaningless

% Split each uniform interval whose midpoint falls inside a window.
dtau = [];
for i = 1:N
    midpoint = (i-0.5)/N;
    inside = midpoint >= starts & midpoint <= stops;
    if any(inside)
        f = max(factors(inside)); % largest applicable factor wins on overlap
        dtau = [dtau, repmat(1/(N*f),1,f)]; %#ok<AGROW>
    else
        dtau = [dtau, 1/N]; %#ok<AGROW>
    end
end

% Normalise so the result sums to 1, correct standing alone.
dtau = dtau/sum(dtau);

end
