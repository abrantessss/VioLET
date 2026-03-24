% developed by: Luís Abrantes 
% plot 2D and 3D UAV position path
close all; clear; clc;
addpath(genpath(pwd))

%% Variables
% CAD model for plot
modelFolder = 'violet_plots/plots/models/easyglider.stl';
scale = 2;
L = 15;
fontsize = 14;

% read state topic
%ros2genmsg("violet_msgs", "BuildConfiguration","fasterbuilds")
bagFolder = 'violet_plots/bags/los2_20260323_162320';
bag = ros2bagreader(bagFolder);
stateTopic = select(bag, 'Topic', '/drone1/fmu/telemetry/state');
stateMsgs = readMessages(stateTopic);

pos = cell2mat(cellfun(@(m) m.position(:)', ...
                       stateMsgs, 'UniformOutput', false));

att = cell2mat(cellfun(@(m) m.attitude(:)', ...
                       stateMsgs, 'UniformOutput', false));

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

%% 3D Path Plot
fig1 = figure('Units','centimeters','Position',[6 6 16 12]);
ax1  = axes('Parent', fig1); hold(ax1,'on'); grid(ax1,'on');
axis(ax1,'equal'); set(ax1, 'ZDir','reverse'); set(ax1, 'YDir','reverse');       
view(ax1, -45, 25); 
xlabel(ax1, 'North [x] (m)', 'FontWeight','bold');
ylabel(ax1, 'East [y] (m)',  'FontWeight','bold');
zlabel(ax1, 'Down [z] (m)',  'FontWeight','bold');

% Load Model
[TR,~,attributes] = stlread(modelFolder);
F  = TR.ConnectivityList;
V0 = TR.Points;
V0(:,3) = -V0(:,3);
V0 = V0 * scale;
h_stl = patch(ax1, 'Faces', F, 'Vertices', V0, 'FaceColor', [0.95 0.95 0.95], 'EdgeColor', 'none');
camlight(ax1,'headlight'); lighting(ax1,'gouraud');

% Plot path
h_path1 = plot3(ax1, pos(:,1), pos(:,2), pos(:,3), '-', 'LineWidth', 2, 'Color', "#ffa500");
h_pos1 = plot3(ax1, pos(1,1), pos(1,2), pos(1,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor','k','MarkerEdgeColor','k');

Vw = (R(:,:,end) * V0.').' + pos(end,:);
h_stl.Vertices = Vw;

h_bx = quiver3(ax1, pos(end,1), pos(end,2), pos(end,3), R(1,1,end)*L, R(2,1,end)*L, R(3,1,end)*L, 'r', 'LineWidth',2, 'MaxHeadSize',0);
h_by = quiver3(ax1, pos(end,1), pos(end,2), pos(end,3), R(1,2,end)*L, R(2,2,end)*L, R(3,2,end)*L, 'g', 'LineWidth',2, 'MaxHeadSize',0);
h_bz = quiver3(ax1, pos(end,1), pos(end,2), pos(end,3), R(1,3,end)*L, R(2,3,end)*L, R(3,3,end)*L, 'b', 'LineWidth',2, 'MaxHeadSize',0);

set(findall(fig1,'-property','FontSize'),'FontSize', fontsize); 
set(gcf,'Renderer','opengl');
legend(ax1, h_path1, {'Vehicle Path'}, 'Location','northwest');
hold off;

%% XY Plane Path Plot
fig2 = figure('Units','centimeters','Position',[6 6 16 12]);
ax2  = axes('Parent',fig2); hold(ax2,'on'); grid(ax2,'on');
axis(ax2,'equal'); set(ax2, 'YDir','reverse'); set(ax2, 'ZDir','reverse');
xlabel(ax2,'North [x] (m)','FontWeight','bold');
ylabel(ax2,'East [y] (m)','FontWeight','bold');

% Paths
hpath1 = plot(ax2, pos(:,1),  pos(:,2),  'k-', 'LineWidth', 2);
plot(ax2, pos(1,1), pos(1,2), 'o', 'MarkerSize', 10, 'MarkerFaceColor','k','MarkerEdgeColor','k');

% Load Model
TR = stlread(modelFolder);
F  = TR.ConnectivityList;
V0 = TR.Points;
V0 = V0 * scale;
V0(:,3) = -V0(:,3);   % keep if needed to fix upside-down

% Draw 5 poses
% Cumulative distance along the vehicle trajectory
dp = diff(pos(:,1:3),1,1);                 % (N-1)x3
s  = [0; cumsum(vecnorm(dp,2,2))];       % Nx1, arc-length from start

s_targets = linspace(0, s(end), 5);  

% Indices closest to those distances (first index where s >= target)
idx = arrayfun(@(st) find(s >= st, 1, 'first'), s_targets);
idx = unique(idx,'stable');           
for ii = 1:numel(idx)
    k  = idx(ii);
    pk = pos(k,:)';
    Rk = R(:,:,k);
    
    psi = atan2(Rk(2,1), Rk(1,1));   % yaw (NED convention)
    Rz  = [ cos(psi) -sin(psi) 0
            sin(psi)  cos(psi) 0
            0         0        1 ];

    Vw = (Rz * V0.').' + pk.';   % rotate + translate

    patch(ax2, 'Faces', F, 'Vertices', Vw, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', 'none');
    % Body axes (same as 3D case)
    bx = Rz(:,1) * L;
    by = Rz(:,2) * L;

    quiver3(ax2, pk(1), pk(2), pk(3), bx(1), bx(2), bx(3), 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0);
    quiver3(ax2, pk(1), pk(2), pk(3), by(1), by(2), by(3), 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0);
end

camlight(ax2,'headlight'); lighting(ax2,'gouraud');
set(findall(fig2,'-property','FontSize'),'FontSize', fontsize); 
set(gcf,'Renderer','opengl');
legend(ax2, hpath1,{'Vehicle Path'}, 'Location','northeast');
hold off;
