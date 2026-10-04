%% 3D Path Plot
fig1 = figure('Units','centimeters','Position',[6 6 figureSize]);
ax1  = axes('Parent', fig1); hold(ax1,'on');
set(ax1, 'ZDir','reverse', 'YDir','reverse');
view(ax1, -45, 25);
xlabel(ax1, 'North [x] (m)');
ylabel(ax1, 'East [y] (m)');
zlabel(ax1, 'Down [z] (m)');

% Paths: yellow vehicle path, dashed black desired path on top
h_path1 = plot3(ax1, pos(:,1), pos(:,2), pos(:,3), '-', 'LineWidth', 3, 'Color', "#ffa500");
h_pd1   = plot3(ax1, pd(:,1), pd(:,2), pd(:,3), 'k--', 'LineWidth', 2);
plot3(ax1, pos(1,1), pos(1,2), pos(1,3), 'o', 'MarkerSize', 10, ...
    'MarkerFaceColor','k', 'MarkerEdgeColor','k');

% Vehicle with body axes at the end of the path
draw_vehicle(ax1, vehicle, pos(end,:), R(:,:,end), L, false);

axis(ax1,'equal');
camlight(ax1,'headlight');
style_axes(ax1, fontsize);
set(fig1,'Renderer','opengl');
legend(ax1, [h_path1, h_pd1], {'Vehicle Path', 'Desired Path'}, 'Location','northwest');
hold(ax1,'off');