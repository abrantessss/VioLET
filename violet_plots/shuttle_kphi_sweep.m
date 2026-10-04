% Compare all matching shuttle trajectory bags in three separate figures:
%   1) XYZ paths with the shuttle model and its body axes
%   2) XY paths with shuttle snapshots and their body axes
%   3) ||ep|| versus time and speeds versus time
% One line colour per sweep value (kphi); the desired path is orange.
% Uses standard controller_results messages, so ros2genmsg is not required.
% Needs shuttle_model.m, shuttle_plot.m and the mesh CSVs (see modelDir).
% Outputs three PDFs, a metrics CSV and a MAT file.
clear; clc; close all;

%% User-adjustable parameters
bagPattern = '*line_kphi*';    % Change line to circle or lemniscate as needed
vehicleNamespace = 'drone1';
fontsize = 14;
lineWidth = 2.8;           % vehicle paths and error curves
desiredLineWidth = 2.0;    % dashed desired path
gridLineWidth = 1.0;       % grid and axes box
figureSize = [18 15];      % [width height] in cm, identical for all three figures/PDFs

% Drone drawing
modelDir      = '';        % folder with shuttle_model.m + CSVs ('' = this folder)
droneBody     = 'mesh';    % 'mesh' = real hull, 'simple' = light parametric body
droneSize     = [];        % drawn rotor-to-rotor span in metres ([] = 12% of path extent)
attitudeBlock = '';        % controller_results label holding [roll pitch yaw] in rad
                           % ('' = draw the drone level with zero yaw)
% Export: the mesh has ~97k triangles, so the two drone figures are exported as
% 600 dpi images inside the PDF. Use droneBody = 'simple' with 'vector' for
% fully vector output.
droneContentType = 'image';

%% Paths and data
plotRoot = fileparts(mfilename('fullpath'));
if isempty(modelDir), modelDir = plotRoot; end
addpath(modelDir);
bagsRoot = fullfile(plotRoot, 'bags');
resultsRoot = fullfile(plotRoot, 'results');
candidates = dir(fullfile(bagsRoot, bagPattern));
candidates = candidates([candidates.isdir]);
candidates = candidates(~ismember({candidates.name}, {'.', '..'}));
if isempty(candidates)
    error('No bag directories matching "%s" in %s.', bagPattern, bagsRoot);
end

runs = struct([]);
for index = 1:numel(candidates)
    bagFolder = fullfile(candidates(index).folder, candidates(index).name);
    fprintf('Reading %s\n', candidates(index).name);
    try
        runData = readRun(bagFolder, vehicleNamespace, attitudeBlock);
        if isempty(runs)
            runs = runData;
        else
            runs(index) = runData;
        end
    catch exception
        error('Cannot read %s:\n%s', bagFolder, getReport(exception, 'extended', 'hyperlinks', 'off'));
    end
end
% Sort numerically, not alphabetically by the filename's kphi token.
[~, order] = sort([runs.kphi]);
runs = runs(order);
colors = lines(numel(runs));
for index = 1:numel(runs)
    % LaTeX label: MATLAB's default TeX interpreter has no \varphi.
    runs(index).label = sprintf('$k_\\varphi = %.6g$', runs(index).kphi);
    if sum([runs.kphi] == runs(index).kphi) > 1
        % Keep repeated experiments identifiable.
        runs(index).label = sprintf('%s (%s)', runs(index).label, ...
            strrep(runs(index).bagName, '_', '\_'));
    end
end
% The run that travels furthest carries the single drone and the desired path.
[~, far] = max(arrayfun(@(r) norm(r.position(end,:) - r.position(1,:)), runs));
farRun = runs(far);

%% Drone model and scale
M = shuttle_model('Body', droneBody);
allV = vertcat(M.parts.V);
modelSpan = max(max(allV(:,1:2)) - min(allV(:,1:2)));
allPts = [vertcat(runs.position); vertcat(runs.pd)];
lo = min(allPts, [], 1); hi = max(allPts, [], 1);
if isempty(droneSize)
    droneSize = 0.12 * max(hi - lo);
end
droneScale = droneSize / modelSpan;
axisLength = 0.6 * droneSize;
pad = droneSize;

%% Figure 1: XYZ paths
fig3d = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 figureSize]);
forceLightTheme(fig3d);
ax3d = axes(fig3d); hold(ax3d, 'on');
for index = 1:numel(runs)
    run = runs(index);
    plot3(ax3d, run.position(:,1), run.position(:,2), run.position(:,3), ...
        'Color', colors(index,:), 'LineWidth', lineWidth, 'DisplayName', run.label);
    plot3(ax3d, run.position(1,1), run.position(1,2), run.position(1,3), 'o', ...
        'MarkerSize', 8, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', ...
        'HandleVisibility', 'off');
end
% Drawn last so the dashes stay visible on top of the vehicle paths.
plot3(ax3d, farRun.pd(:,1), farRun.pd(:,2), farRun.pd(:,3), 'k--', ...
    'LineWidth', desiredLineWidth, 'DisplayName', 'Desired Path');
drawDrone(ax3d, M, farRun.position(end,:), farRun.rpy(end,:), droneScale, axisLength, true);
set(ax3d, 'YDir', 'reverse', 'ZDir', 'reverse');
xlim(ax3d, [lo(1)-pad hi(1)+pad]); ylim(ax3d, [lo(2)-pad hi(2)+pad]);
zHalf = max((hi(3)-lo(3))/2 + pad, 0.25*max(hi(1:2)-lo(1:2)));
zlim(ax3d, (lo(3)+hi(3))/2 + [-zHalf zHalf]);
daspect(ax3d, [1 1 1]);
view(ax3d, -45, 25);
relight(ax3d);
xlabel(ax3d, 'North [x] (m)'); ylabel(ax3d, 'East [y] (m)'); zlabel(ax3d, 'Down [z] (m)');
styleAxes(ax3d, fontsize, gridLineWidth);
legend(ax3d, 'Location', 'northwest', 'Interpreter', 'latex');

%% Figure 2: XY paths
fig2d = figure('Color', 'w', 'Units', 'centimeters', 'Position', [4 2 figureSize]);
forceLightTheme(fig2d);
ax2d = axes(fig2d); hold(ax2d, 'on');
for index = 1:numel(runs)
    run = runs(index);
    plot(ax2d, run.position(:,1), run.position(:,2), ...
        'Color', colors(index,:), 'LineWidth', lineWidth, 'DisplayName', run.label);
end
plot(ax2d, farRun.pd(:,1), farRun.pd(:,2), 'k--', ...
    'LineWidth', desiredLineWidth, 'DisplayName', 'Desired Path');
% Drawn in the z = 0 plane so that it sits with the 2-D path lines.
drawDrone(ax2d, M, [farRun.position(end,1:2) 0], farRun.rpy(end,:), droneScale, axisLength, false);
% ZDir reversed too: the camera then looks down on the top of the vehicle (NED).
set(ax2d, 'YDir', 'reverse', 'ZDir', 'reverse');
xlim(ax2d, [lo(1)-pad hi(1)+pad]); ylim(ax2d, [lo(2)-pad hi(2)+pad]);
zlim(ax2d, [-2*droneSize 2*droneSize]);
daspect(ax2d, [1 1 1]);
view(ax2d, 0, 90);
relight(ax2d);
xlabel(ax2d, 'North [x] (m)'); ylabel(ax2d, 'East [y] (m)');
styleAxes(ax2d, fontsize, gridLineWidth);
legend(ax2d, 'Location', 'northeast', 'Interpreter', 'latex');

%% Figure 3: norm error
figErr = figure('Color', 'w', 'Units', 'centimeters', 'Position', [6 2 figureSize]);
forceLightTheme(figErr);
axError = axes(figErr); hold(axError, 'on');
tEnd = max(arrayfun(@(r) r.time(end), runs));
for index = 1:numel(runs)
    run = runs(index);
    plot(axError, run.time, run.errorNorm, ...
        'Color', colors(index,:), 'LineWidth', lineWidth, 'DisplayName', run.label);
end
xlabel(axError, 'Time (s)'); ylabel(axError, 'Norm Error (m)');
xlim(axError, [0 tEnd]);
ylim(axError, [0 Inf]);
styleAxes(axError, fontsize, gridLineWidth);
legend(axError, 'Location', 'northeast', 'Interpreter', 'latex');

%% Save results
% Unique output folder keeps earlier sweep results intact.
sweepName = ['shuttle_' regexprep(bagPattern, '[^a-zA-Z0-9_-]', '') ...
             '_sweep_' char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'))];
resultFolder = fullfile(resultsRoot, sweepName);
mkdir(resultFolder);
metrics = table(string({runs.bagName})', [runs.kphi]', ...
    arrayfun(@(r) numel(r.time), runs)', ...
    arrayfun(@(r) r.time(end), runs)', ...
    arrayfun(@(r) sqrt(trapz(r.time, r.errorNorm.^2) / r.time(end)), runs)', ...
    arrayfun(@(r) max(r.errorNorm), runs)', ...
    arrayfun(@(r) r.errorNorm(end), runs)', ...
    'VariableNames', {'Bag', 'Kphi', 'Samples', 'Duration_s', ...
                      'TimeWeightedRMSError_m', 'MaxError_m', 'FinalError_m'});
pdf3d  = fullfile(resultFolder, [sweepName '_xyz.pdf']);
pdf2d  = fullfile(resultFolder, [sweepName '_xy.pdf']);
pdfErr = fullfile(resultFolder, [sweepName '_error.pdf']);
csvFile = fullfile(resultFolder, [sweepName '_metrics.csv']);
matFile = fullfile(resultFolder, [sweepName '_data.mat']);
% Fixed page size (no tight cropping), so the three PDFs are identical in size.
savePdf(fig3d, pdf3d, figureSize, droneContentType);
savePdf(fig2d, pdf2d, figureSize, droneContentType);
savePdf(figErr, pdfErr, figureSize, 'vector');
writetable(metrics, csvFile);
save(matFile, 'runs', 'metrics', 'bagPattern', 'vehicleNamespace');
disp(metrics);
fprintf('Saved plots:\n  %s\n  %s\n  %s\nSaved metrics: %s\nSaved MATLAB data: %s\n', ...
        pdf3d, pdf2d, pdfErr, csvFile, matFile);

%% Local functions
function drawDrone(ax, M, pos, rpy, scale, axisLength, drawZAxis)
    % Shuttle model (x forward, y left, z up) placed in the NED plot frame:
    % centre on the CoM, scale, flip FLU -> FRD, rotate by ZYX attitude, translate.
    rotation = makehgtform('zrotate', rpy(3)) * makehgtform('yrotate', rpy(2)) * ...
               makehgtform('xrotate', rpy(1));
    outer = hgtransform('Parent', ax);
    h = shuttle_plot(M, 'Parent', outer);
    set(h.patches, 'HandleVisibility', 'off');
    set(outer, 'Matrix', makehgtform('translate', pos) * rotation * ...
        makehgtform('xrotate', pi) * makehgtform('scale', scale) * ...
        makehgtform('translate', -M.com));
    % Body axes: x forward (red), y right (green), z down (blue).
    R = rotation(1:3, 1:3);
    axisColors = {[0.9 0 0], [0 0.8 0], [0 0 0.9]};
    lift = [0 0 0];
    if ~drawZAxis
        lift = [0 0 -scale]; % towards the top-view camera, above the hull
    end
    for k = 1:(2 + drawZAxis)
        tip = pos + axisLength * R(:,k)';
        plot3(ax, [pos(1) tip(1)] + lift(1), [pos(2) tip(2)] + lift(2), ...
            [pos(3) tip(3)] + lift(3), 'Color', axisColors{k}, ...
            'LineWidth', 2.5, 'HandleVisibility', 'off');
    end
end

function savePdf(fig, file, sizeCm, contentType)
    % print with an explicit paper size; exportgraphics would crop each figure
    % to its own content and give three different page sizes.
    set(fig, 'PaperUnits', 'centimeters', 'PaperSize', sizeCm, ...
        'PaperPosition', [0 0 sizeCm], 'InvertHardcopy', 'off');
    print(fig, file, '-dpdf', ['-' contentType], '-r600');
end

function forceLightTheme(fig)
    % R2025a+ figures follow the desktop theme; in dark mode the black dashed
    % path and dark grid would sit on a dark axes background.
    if ~isMATLABReleaseOlderThan('R2025a')
        theme(fig, 'light');
    end
end

function relight(ax)
    % Replace shuttle_plot's lights (set for a z-up frame) with a headlight.
    delete(findobj(ax, 'Type', 'light'));
    camlight(ax, 'headlight');
end

function styleAxes(ax, fontsize, gridLineWidth)
    box(ax, 'on');
    ax.LineWidth = gridLineWidth; % grid lines and axes box
    % Explicit, darker grid: the default (alpha 0.15) all but vanishes on export.
    set(ax, 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', 'GridLineStyle', '-', ...
        'GridColor', [0.15 0.15 0.15], 'GridAlpha', 0.35, 'Layer', 'bottom');
    ax.FontSize = fontsize;
    ax.XLabel.FontWeight = 'bold';
    ax.YLabel.FontWeight = 'bold';
    ax.ZLabel.FontWeight = 'bold';
    ax.Toolbar.Visible = 'off';
end

function run = readRun(bagFolder, vehicleNamespace, attitudeBlock)
    bag = ros2bagreader(bagFolder);
    topic = sprintf('/%s/fmu/telemetry/controller_results', vehicleNamespace);
    selection = select(bag, 'Topic', topic);
    messages = readMessages(selection);
    if isempty(messages)
        error('No messages on %s. This script requires the mission results recorder.', topic);
    end
    time = selection.MessageList.Time;
    if isdatetime(time) || isduration(time)
        time = seconds(time - time(1));
    else
        time = double(time);
        time = time - time(1);
    end
    time = time(:);
    position = zeros(numel(messages), 3);
    pd = position; ep = position; rpy = position;
    kphi = zeros(numel(messages), 1);
    vd = kphi;
    % Decode labelled blocks on every sample, independent of block ordering.
    for sample = 1:numel(messages)
        msg = messages{sample};
        position(sample,:) = readBlock(msg, 'position', 3);
        pd(sample,:) = readBlock(msg, 'pd', 3);
        ep(sample,:) = readBlock(msg, 'ep', 3);
        kphi(sample) = readBlock(msg, 'kphi', 1);
        vd(sample) = readBlock(msg, 'vd', 1);
        if ~isempty(attitudeBlock)
            rpy(sample,:) = readBlock(msg, attitudeBlock, 3);
        end
    end
    valid = isfinite(time) & all(isfinite(position), 2) & all(isfinite(pd), 2) ...
            & all(isfinite(ep), 2) & isfinite(kphi) & isfinite(vd) & all(isfinite(rpy), 2);
    % Discard any queued staging-waypoint samples preceding the trajectory command.
    firstActive = find(valid & vd > 0, 1, 'first');
    if ~isempty(firstActive)
        valid(1:firstActive-1) = false;
    end
    time = time(valid); position = position(valid,:); pd = pd(valid,:);
    ep = ep(valid,:); kphi = kphi(valid); vd = vd(valid); rpy = rpy(valid,:);
    if numel(time) < 2
        error('Fewer than two finite trajectory samples.');
    end
    if any(abs(kphi - kphi(1)) > max(1e-12, abs(kphi(1))*1e-9))
        error('kphi changes within this bag; record each sweep setting separately.');
    end
    [time, indices] = unique(time, 'last');
    time = time - time(1);
    if numel(time) < 2 || time(end) <= 0
        error('Trajectory samples must span a positive time interval.');
    end
    position = position(indices,:); pd = pd(indices,:); ep = ep(indices,:);
    vd = vd(indices); rpy = rpy(indices,:);
    [~, bagStem, bagExtension] = fileparts(bagFolder);
    bagName = [bagStem bagExtension]; % Preserve decimal kphi in directory names.
    run = struct('bagFolder', bagFolder, 'bagName', bagName, 'kphi', kphi(1), ...
        'time', time, 'position', position, 'pd', pd, 'ep', ep, ...
        'errorNorm', vecnorm(ep, 2, 2), 'rpy', rpy, 'vd', vd, 'label', '');
end

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