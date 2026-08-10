function [tau,dtau,N] = getMeshIntervals(S)
% --------------------------------------------------------------------------
% getMeshIntervals
%   Returns the normalised mesh of the collocation grid. The mesh is uniform
%   unless S.solver.mesh_intervals is set.
%
% INPUT:
%   - S -
%   * setting structure S
%
% OUTPUT:
%   - tau -
%   * normalised mesh points, from 0 to 1 (1 x N+1)
%
%   - dtau -
%   * normalised mesh interval durations, sums to 1 (1 x N)
%
%   - N -
%   * number of mesh intervals
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

if isempty(S.solver.mesh_intervals)
    N = S.solver.N_meshes;
    dtau = ones(1,N)/N;
else
    dtau = S.solver.mesh_intervals;
    N = numel(dtau);
end

tau = [0, cumsum(dtau)];
tau(end) = 1; % remove round-off on the last point

end
