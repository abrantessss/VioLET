% developed by: Luís Abrantes
% plot 2D and 3D UAV position path
% Reads the standard controller_results messages (same as the sweep script),
% so ros2genmsg is not required.

close all; clear; clc;
addpath(genpath(pwd))

%% Variables
% 3D model for plot (shuttle_model.m + mesh CSVs must be on the path)
M = shuttle_model();
vehicle = struct('M', M, 'scale', 15, 'centre', M.com, ...
                 'toFRD', diag([1 -1 -1]));   % model frame: x forward, y left, z up
L = 6;                 % body-axis length (m)
nPoses = 5;             % vehicles drawn in the XY plot
fontsize = 14;
figureSize = [16 12];   % [width height] cm, same for the three figures

bagFolder = 'bags/mellinger_lemniscate_*';   % full name, or * for the timestamp part
vehicleNamespace = 'drone1';

%% Read bag
% Resolve the bag folder from the current folder, this script's folder or its parent,
% so the script works whichever folder MATLAB is in.
scriptDir = fileparts(mfilename('fullpath'));
roots = {pwd, scriptDir, fileparts(scriptDir)};
found = find(cellfun(@(r) ~isempty(dir(fullfile(r, bagFolder))), roots), 1);
if isempty(found)
    error('Bag folder "%s" not found in:\n  %s', bagFolder, strjoin(roots, [newline '  ']));
end
match = dir(fullfile(roots{found}, bagFolder));
match = match([match.isdir] & ~ismember({match.name}, {'.', '..'}));
if isempty(match)                      % bagFolder was an exact folder name
    bagFolder = fullfile(roots{found}, bagFolder);
else                                   % wildcard: take the most recent match
    [~, newest] = max([match.datenum]);
    bagFolder = fullfile(match(newest).folder, match(newest).name);
end
fprintf('Reading %s\n', bagFolder);

bag = ros2bagreader(bagFolder);
topic = sprintf('/%s/fmu/telemetry/controller_results', vehicleNamespace);
selection = select(bag, 'Topic', topic);
messages = readMessages(selection);
if isempty(messages)
    error('No messages on %s. This script requires the mission results recorder.', topic);
end
t = selection.MessageList.Time;
if isdatetime(t) || isduration(t)
    t = seconds(t - t(1));
else
    t = double(t);
end
t = t(:);

n   = numel(messages);
pos = zeros(n,3); v = pos; pd = pos; dpd_dgamma = pos; eR = pos;
gamma_dot = zeros(n,1); vd = gamma_dot;
R = zeros(3,3,n);
% Decode labelled blocks on every sample, independent of block ordering.
for k = 1:n
    msg = messages{k};
    pos(k,:)        = readBlock(msg, 'position', 3);
    v(k,:)          = readBlock(msg, 'velocity', 3);
    pd(k,:)         = readBlock(msg, 'pd', 3);
    dpd_dgamma(k,:) = readBlock(msg, 'dpd_dgamma', 3);
    gamma_dot(k)    = readBlock(msg, 'gamma_dot', 1);
    vd(k)           = readBlock(msg, 'vd', 1);
    eR(k,:)         = readBlock(msg, 'eR', 3);
    R(:,:,k)        = reshape(readBlock(msg, 'R', 9), 3, 3)';   % stored row-major
end

valid = isfinite(t) & all(isfinite([pos v pd dpd_dgamma eR gamma_dot vd]), 2) ...
        & squeeze(all(isfinite(R), [1 2]));
% Discard any queued staging-waypoint samples preceding the trajectory command.
firstActive = find(valid & vd > 0, 1, 'first');
if ~isempty(firstActive)
    valid(1:firstActive-1) = false;
end
t = t(valid);
[t, keep] = unique(t, 'last');
idx = find(valid); idx = idx(keep);
pos = pos(idx,:); v = v(idx,:); pd = pd(idx,:); dpd_dgamma = dpd_dgamma(idx,:); eR = eR(idx,:);
gamma_dot = gamma_dot(idx); vd = vd(idx); R = R(:,:,idx);
if numel(t) < 2
    error('Fewer than two finite trajectory samples.');
end
t = t - t(1);

% Path-parameter rates -> speeds along the path (m/s), as in the original script.
gamma_dot = gamma_dot .* vecnorm(dpd_dgamma, 2, 2);
vd        = vd        .* vecnorm(dpd_dgamma, 2, 2);

%% Plots
pos3d
pos2d
poserror
saveplots

%% Local functions
function values = readBlock(msg, label, expectedSize)
    offset = 0;
    for index = 1:numel(msg.layout.dim)
        dim = msg.layout.dim(index);
        count = double(dim.size);
        if strcmp(dim.label, label)
            if count ~= expectedSize || offset + count > numel(msg.data)
                error('Invalid controller_results block "%s".', label);
            end
            values = double(msg.data(offset + (1:count)));
            values = values(:)';
            return;
        end
        offset = offset + count;
    end
    error('Missing controller_results block "%s".', label);
end