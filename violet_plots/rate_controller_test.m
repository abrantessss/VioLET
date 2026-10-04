%% ========================================================================
%  rate_controller_test.m
%
%  Reads a shuttle or combined rate-controller rosbag, plots commanded and
%  measured body rates, calculates step-response metrics, and saves the
%  results as a vector PDF, CSV, and MAT file.
%
%  Custom ROS 2 messages must be generated once before reading the bag:
%    ros2genmsg(fullfile(repoRoot, 'violet_msgs'), ...
%               'BuildConfiguration', 'fasterbuilds')
%  Then add the generated message folder to the MATLAB path.
% ========================================================================
clear; clc; close all;

%% ---------------------- User-adjustable parameters ---------------------
fontsize = 20;
settlingTolerance = 0.05;       % Settling band [rad/s]

% Leave bagFolder empty to use the newest matching bag. Examples:
%   bagPattern = 'shuttle_*';
%   bagPattern = 'combined_*';
% Or set bagFolder to a specific bag directory.
bagFolder = '';
bagPattern = 'combined_*';

shuttleNamespace = 'drone1';

%% ------------------------- Paths and rosbag ----------------------------
plotRoot = fileparts(mfilename('fullpath'));
repoRoot = fileparts(plotRoot);
bagsRoot = fullfile(plotRoot, 'bags');
resultsRoot = fullfile(plotRoot, 'results');
addpath(genpath(repoRoot));

if ~isfolder(resultsRoot)
    mkdir(resultsRoot);
end

if isempty(bagFolder)
    candidates = dir(fullfile(bagsRoot, bagPattern));
    candidates = candidates([candidates.isdir]);
    if isempty(candidates)
        error('No bag directories matching "%s" were found in %s.', ...
              bagPattern, bagsRoot);
    end
    [~, newestIndex] = max([candidates.datenum]);
    bagFolder = fullfile(candidates(newestIndex).folder, ...
                         candidates(newestIndex).name);
elseif ~isfolder(bagFolder)
    bagFolder = fullfile(repoRoot, bagFolder);
end

if ~isfolder(bagFolder)
    error('Rosbag directory does not exist: %s', bagFolder);
end

[~, bagName] = fileparts(bagFolder);
resultFolder = fullfile(resultsRoot, bagName);
if ~isfolder(resultFolder)
    mkdir(resultFolder);
end

rateTopicName = sprintf('/%s/controller/in/rate_setpoint', shuttleNamespace);
stateTopicName = sprintf('/%s/fmu/telemetry/state', shuttleNamespace);

try
    bag = ros2bagreader(bagFolder);
    rateSelection = select(bag, 'Topic', rateTopicName);
    stateSelection = select(bag, 'Topic', stateTopicName);
    rateMsgs = readMessages(rateSelection);
    stateMsgs = readMessages(stateSelection);
catch exception
    error(['Could not read the ROS 2 messages. Generate violet_msgs with ', ...
           'ros2genmsg and add the generated folder to the MATLAB path.\n', ...
           'Original error: %s'], exception.message);
end

if isempty(rateMsgs)
    error('No messages found on %s.', rateTopicName);
end
if isempty(stateMsgs)
    error('No messages found on %s.', stateTopicName);
end

%% ---------------------------- Read data --------------------------------
% VehicleRatesSetpoint timestamp is in microseconds.
tSetpoint = cellfun(@(m) double(m.timestamp) * 1e-6, rateMsgs);
tSetpoint = tSetpoint(:);
rateSetpoint = cell2mat(cellfun( ...
    @(m) [double(m.roll), double(m.pitch), double(m.yaw)], ...
    rateMsgs, 'UniformOutput', false));

% State uses the standard ROS Header timestamp.
tMeasured = cellfun(@(m) double(m.header.stamp.sec) + ...
                         double(m.header.stamp.nanosec) * 1e-9, stateMsgs);
tMeasured = tMeasured(:);
rateMeasured = cell2mat(cellfun( ...
    @(m) double(m.angular_velocity(:)'), ...
    stateMsgs, 'UniformOutput', false));

validSetpoint = isfinite(tSetpoint) & all(isfinite(rateSetpoint), 2);
validMeasured = isfinite(tMeasured) & all(isfinite(rateMeasured), 2);
tSetpoint = tSetpoint(validSetpoint);
rateSetpoint = rateSetpoint(validSetpoint, :);
tMeasured = tMeasured(validMeasured);
rateMeasured = rateMeasured(validMeasured, :);

if isempty(tSetpoint) || isempty(tMeasured)
    error('The selected topics contain no finite rate data.');
end

% Remove repeated timestamps before interpolation. This can occur because
% the Python test publishes setpoints faster than the state update rate.
[tSetpoint, uniqueIndices] = unique(tSetpoint, 'last');
rateSetpoint = rateSetpoint(uniqueIndices, :);
[tMeasured, uniqueIndices] = unique(tMeasured, 'last');
rateMeasured = rateMeasured(uniqueIndices, :);

t0 = min([tSetpoint(1), tMeasured(1)]);
tSetpoint = tSetpoint - t0;
tMeasured = tMeasured - t0;

%% -------------------- Identify tested axis and step ---------------------
axisNames = {'roll', 'pitch', 'yaw'};
axisLabels = {'Roll', 'Pitch', 'Yaw'};

axisToken = regexp(bagName, '_(roll|pitch|yaw)_', 'tokens', 'once');
if isempty(axisToken)
    [~, testedAxis] = max(max(abs(rateSetpoint), [], 1));
else
    testedAxis = find(strcmp(axisNames, axisToken{1}), 1);
end

active = abs(rateSetpoint(:, testedAxis)) > 1e-6;
stepStartIndex = find(active, 1, 'first');
if isempty(stepStartIndex)
    error('No non-zero %s rate step was found.', axisNames{testedAxis});
end

returnOffset = find(~active(stepStartIndex + 1:end), 1, 'first');
if isempty(returnOffset)
    stepEndIndex = numel(tSetpoint);
else
    stepEndIndex = stepStartIndex + returnOffset;
end

stepStartTime = tSetpoint(stepStartIndex);
stepEndTime = tSetpoint(stepEndIndex);
stepTarget = median(rateSetpoint(stepStartIndex:stepEndIndex - 1, testedAxis));

%% ---------------------- Step-response metrics --------------------------
beforeStep = find(tMeasured < stepStartTime, 1, 'last');
if isempty(beforeStep)
    initialValue = rateMeasured(1, testedAxis);
else
    initialValue = rateMeasured(beforeStep, testedAxis);
end

stepSamples = tMeasured >= stepStartTime & tMeasured < stepEndTime;
stepTimes = tMeasured(stepSamples) - stepStartTime;
stepValues = rateMeasured(stepSamples, testedAxis);
if numel(stepValues) < 2
    error('Not enough measured samples were recorded during the step.');
end

stepSize = stepTarget - initialValue;
direction = sign(stepSize);
if direction == 0
    error('The detected step has zero amplitude.');
end

overshoot = 100 * max(0, max(direction * (stepValues - stepTarget))) / abs(stepSize);
t10 = firstCrossing(stepTimes, stepValues, initialValue + 0.1 * stepSize, direction);
t90 = firstCrossing(stepTimes, stepValues, initialValue + 0.9 * stepSize, direction);
if isnan(t10) || isnan(t90)
    riseTime = NaN;
else
    riseTime = t90 - t10;
end

outsideBand = find(abs(stepValues - stepTarget) > settlingTolerance, 1, 'last');
if isempty(outsideBand)
    settlingTime = 0;
elseif outsideBand < numel(stepTimes)
    settlingTime = stepTimes(outsideBand + 1);
else
    settlingTime = NaN;
end

steadyCount = max(1, round(0.1 * numel(stepValues)));
steadyValue = mean(stepValues(end - steadyCount + 1:end));
steadyStateError = stepTarget - steadyValue;

metrics = table(string(bagName), string(axisNames{testedAxis}), ...
    stepTarget, initialValue, max(direction * stepValues) * direction, ...
    overshoot, riseTime, settlingTime, steadyValue, steadyStateError, ...
    'VariableNames', {'Bag', 'Axis', 'Setpoint_rad_s', 'Initial_rad_s', ...
    'Peak_rad_s', 'Overshoot_percent', 'RiseTime_s', 'SettlingTime_s', ...
    'SteadyValue_rad_s', 'SteadyStateError_rad_s'});

%% ======================= RATE RESPONSE FIGURE ==========================
fig = figure;
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for axis = 1:3
    ax = nexttile(layout);
    axesHandles(axis) = ax;

    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    ax.FontSize = fontsize;

    stairs(ax, tSetpoint, rateSetpoint(:, axis), 'LineWidth', 2, ...
        'DisplayName', sprintf('%s setpoint', lower(axisLabels{axis})));
    plot(ax, tMeasured, rateMeasured(:, axis), 'LineWidth', 2, ...
        'DisplayName', sprintf('%s measured', lower(axisLabels{axis})));
    ylabel(ax, sprintf('$%s$ [rad/s]', axisNames{axis}(1)), ...
        'Interpreter', 'latex');
    ylim(ax, [-0.2 0.8]);
    xlim(ax, [0 30]);
    legend(ax, 'Location', 'northeast', 'Interpreter', 'latex');
    grid(ax, 'minor');
end

xlabel(axesHandles(3), 'Time [s]', ...
    'Interpreter', 'latex', ...
    'FontSize', fontsize);
set(findall(fig, '-property', 'FontSize'), 'FontSize', fontsize);
set(fig, 'Renderer', 'opengl');

%% ---------------------------- Save results -----------------------------
pdfFile = fullfile(resultFolder, [bagName '_rate_response.pdf']);
csvFile = fullfile(resultFolder, [bagName '_metrics.csv']);
matFile = fullfile(resultFolder, [bagName '_rate_data.mat']);

exportgraphics(fig, pdfFile, 'ContentType', 'vector');
writetable(metrics, csvFile);
save(matFile, 'bagFolder', 'bagName', 'tSetpoint', 'rateSetpoint', ...
    'tMeasured', 'rateMeasured', 'testedAxis', 'stepStartTime', ...
    'stepEndTime', 'settlingTolerance', 'metrics');

disp(metrics);
fprintf('Saved vector plot: %s\n', pdfFile);
fprintf('Saved metrics:     %s\n', csvFile);
fprintf('Saved MATLAB data: %s\n', matFile);

%% --------------------------- Local functions ---------------------------
function crossingTime = firstCrossing(time, values, threshold, direction)
    index = find(direction * (values - threshold) >= 0, 1, 'first');
    if isempty(index)
        crossingTime = NaN;
        return;
    end
    if index == 1 || values(index) == values(index - 1)
        crossingTime = time(index);
        return;
    end
    fraction = (threshold - values(index - 1)) / ...
               (values(index) - values(index - 1));
    crossingTime = time(index - 1) + fraction * ...
                   (time(index) - time(index - 1));
end

function name = vehicleNameFromBag(bagName)
    if startsWith(bagName, 'combined_')
        name = 'Combined vehicle';
    elseif startsWith(bagName, 'shuttle_')
        name = 'Shuttle';
    else
        name = 'Vehicle';
    end
end
