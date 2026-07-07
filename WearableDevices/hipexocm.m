function [exo] = hipexocm(init, settings_orthosis)
% --------------------------------------------------------------------------
% Hip exo with 2 hip flexion/extension actuators
% type Cubemars AK80-8
% Backdrive (Coulomb-like) torque: 0.75 Nm
% Rotor inertia: 6.086e-05 kgm^2
% Reduction: 8

%
% INPUT:
%   - init -
%   * struct with information used to initialise the Orthosis object.
% 
%   - settings_orthosis -
%   * struct with information about this orthosis, containing the fields:
%       - function_name = hipexo  i.e. name of this function   
%       - gain: EMG gain
%       - left_right: char, 'l' or 'r'
%       - dynamics: struct with lower and upper limits for opti controls and
%           states
%   Values are set via S.orthosis.settings{i} in main.m, with i the index
%   of the orthosis.
%
%
% OUTPUT:
%   - exo -
%   * an object of the class Orthosis
% 
% Original author: Sander De Groof
% Original date: 14/November/2025
% --------------------------------------------------------------------------

% create Orthosis object
exo = Orthosis('exo',init, true);

% read settings that were passed from main.m
if isfield(settings_orthosis,'isFullGaitCycle')
    isFullGaitCycle = settings_orthosis.isFullGaitCycle;
else
    isFullGaitCycle = false;
end

side = settings_orthosis.left_right; % 'l' for left or 'r' for right
% gain = settings_orthosis.gain;
% 
% emg1 = exo.var_muscle(['glut_max1_',side]);
% emg2 = exo.var_muscle(['glut_max2_',side]);
% emg3 = exo.var_muscle(['glut_max3_',side]);
% 
% emg = (emg1 + emg2 + emg3)/3;

%u_hipfl_emg = -(emg-0.05)*gain; % only use activation above lower bound (0.05)

%test time variable control
% timebase = 0:init.Nmesh;
% if strcmp(side,'l')
%     TimeVarControl = 10*sin(2*pi*timebase/timebase(end)); %10Nm torque in the middle of cycle
% else
%     TimeVarControl = 10*sin(2*pi*timebase/timebase(end)+pi); %10Nm torque in the middle of cycle
% end

% pos = var_coord(['hip_flexion_',side],'pos'); % hip position in rad
vel = exo.var_coord(['hip_flexion_',side],'vel'); % hip velocity in rad/s
acc = exo.var_coord(['hip_flexion_',side],'acc'); % hip acceleration in rad/s^2

coulomb_friction_torque = 0.75 * atan(1000*vel) *2/pi;
acceleration_torque = acc * 8 * 8 * 6.086e-05;

% Simple first order dynamics dx/dt = (u-x)/tau
%tc_tau = 0.05; % time constant in seconds
%state_x = exo.var_opti(['state_x_' side '_side'],'state',[settings_orthosis.dynamics.xl settings_orthosis.dynamics.xu]);
control_u = exo.var_opti(['control_u_' side '_side'],'control',[settings_orthosis.dynamics.ul settings_orthosis.dynamics.uu]);

%control_exo = u_hipfl_emg + control_u; % to make it interesting
%control_exo = control_u;

totaltorque = control_u - coulomb_friction_torque - acceleration_torque;

%exo.addDynamics((control_exo-state_x)/tc_tau,['state_x_' side '_side']); %state and control
%exo.addDynamics([(control_exo-state_x)/tc_tau;(control_exo-state_x2)/tc_tau],{['state_x_' side '_side'],['state_x2_' side '_side']}); %state and control
%exo.addDynamics((emg-state_x)/tc_tau,['state_x_' side '_side']); %state, but no control
%exo.addCoordForce(state_x,['hip_flexion_',side]);
%exo.addCoordForce(state_x+TimeVarControl,['hip_flexion_',side]);
%exo.addCoordForce(control_u,['hip_flexion_',side]); % add control straight away
exo.addCoordForce(totaltorque,['hip_flexion_',side]); % add control straight away

%exo.addVarToPostProcessing(state_x,['state_x_' side '_side'])
%exo.addVarToPostProcessing(control_exo,['control_u_' side '_side'])
%exo.addVarToPostProcessing(control_u,['control_u_' side '_side'])
end

