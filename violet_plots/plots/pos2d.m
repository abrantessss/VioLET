%% XY Plane Path Plot
fig2 = figure('Units','centimeters','Position',[6 6 figureSize]);
ax2  = axes('Parent',fig2); hold(ax2,'on');
set(ax2, 'YDir','reverse', 'ZDir','reverse');
xlabel(ax2,'North [x] (m)');
ylabel(ax2,'East [y] (m)');

% Paths: yellow vehicle path, dashed black desired path on top
hpath1 = plot(ax2, pos(:,1), pos(:,2), '-', 'LineWidth', 3, 'Color', "#ffa500");
hpd1   = plot(ax2, pd(:,1), pd(:,2), 'k--', 'LineWidth', 2);

% Draw the vehicle at poses equally spaced in distance along its path
dp = diff(pos(:,1:3),1,1);
s  = [0; cumsum(vecnorm(dp,2,2))];           % arc-length from start
s_targets = linspace(0, s(end), nPoses);
idx = arrayfun(@(st) find(s >= st, 1, 'first'), s_targets);
idx = unique(idx,'stable');
for ii = 1:numel(idx)
    k   = idx(ii);
    psi = atan2(R(2,1,k), R(1,1,k));         % yaw only (top view)
    Rz  = [ cos(psi) -sin(psi) 0
            sin(psi)  cos(psi) 0
            0         0        1 ];
    draw_vehicle(ax2, vehicle, pos(k,:), Rz, L, true);
end

axis(ax2,'equal');
view(ax2, 0, 90);
camlight(ax2,'headlight');
style_axes(ax2, fontsize);
set(fig2,'Renderer','opengl');
legend(ax2, [hpath1, hpd1], {'Vehicle Path', 'Desired Path'}, 'Location','northeast');
hold(ax2,'off');