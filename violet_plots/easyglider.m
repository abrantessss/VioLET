% developed by: Luís Abrantes 
% plot 2D and 3D UAV position path
close all; clear; clc;
addpath(genpath(pwd))

%% Variables
% 3D model for plot (easyglider_model.m + easyglider_mesh.json must be on the path)
M = easyglider_model();
vehicle = struct('M', M, 'scale', 5, 'centre', M.cg_body(:)', ...
                 'toFRD', diag([1 1 -1]));    % model frame: x forward, y right, z up
L = 8;                  % body-axis length (m)
nPoses = 5;             % vehicles drawn in the XY plot
fontsize = 14;
figureSize = [16 12];   % [width height] cm, same for the three figures

% read state topic
%ros2genmsg("violet_msgs", "BuildConfiguration","fasterbuilds")
bagFolder = 'violet_plots/bags/los4_20260521_115151';
bag = ros2bagreader(bagFolder);
stateTopic = select(bag, 'Topic', '/drone1/plots/data');
stateMsgs = readMessages(stateTopic);

t = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec) * 1e-9, ...
            stateMsgs);

pos = cell2mat(cellfun(@(m) m.position(:)', ...
                       stateMsgs, 'UniformOutput', false));

v = cell2mat(cellfun(@(m) m.inertial_velocity(:)', ...
                     stateMsgs, 'UniformOutput', false));

pd = cell2mat(cellfun(@(m) m.pd(:)', ...
                      stateMsgs, 'UniformOutput', false));

dpd_dgamma = cell2mat(cellfun(@(m) m.dpd_dgamma(:)', ...
                              stateMsgs, 'UniformOutput', false));

att = cell2mat(cellfun(@(m) m.attitude(:)', ...
                       stateMsgs, 'UniformOutput', false));

w = cell2mat(cellfun(@(m) m.angular_velocity(:)', ...
                     stateMsgs, 'UniformOutput', false));

gamma_dot = cellfun(@(m) double(m.gamma), stateMsgs) .* vecnorm(dpd_dgamma, 2, 2);
vd = cellfun(@(m) double(m.vd), stateMsgs);

n = size(att,1);
R = zeros(3,3,n);

for k = 1:n
    phi   = att(k,1);   % roll
    theta = att(k,2);   % pitch
    psi   = att(k,3);   % yaw

    cphi = cos(phi);   sphi = sin(phi);
    cth  = cos(theta); sth  = sin(theta);
    cpsi = cos(psi);   spsi = sin(psi);

    R(:,:,k) = [ ...
        cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
        spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
        -sth,      cth*sphi,                   cth*cphi];
end

%% Plots
pos3d
pos2d
poserror
saveplots