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
h_pd1 = plot3(ax1, pd(:,1), pd(:,2), pd(:,3), '--', 'LineWidth', 2, 'Color', "#1f77b4");
h_pos1 = plot3(ax1, pos(1,1), pos(1,2), pos(1,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor','k','MarkerEdgeColor','k');

Vw = (R(:,:,end) * V0.').' + pos(end,:);
h_stl.Vertices = Vw;

h_bx = quiver3(ax1, pos(end,1), pos(end,2), pos(end,3), R(1,1,end)*L, R(2,1,end)*L, R(3,1,end)*L, 'r', 'LineWidth',2, 'MaxHeadSize',0);
h_by = quiver3(ax1, pos(end,1), pos(end,2), pos(end,3), R(1,2,end)*L, R(2,2,end)*L, R(3,2,end)*L, 'g', 'LineWidth',2, 'MaxHeadSize',0);
h_bz = quiver3(ax1, pos(end,1), pos(end,2), pos(end,3), R(1,3,end)*L, R(2,3,end)*L, R(3,3,end)*L, 'b', 'LineWidth',2, 'MaxHeadSize',0);

set(findall(fig1,'-property','FontSize'),'FontSize', fontsize); 
set(gcf,'Renderer','opengl');
legend(ax1, [h_path1, h_pd1], {'Vehicle Path', 'Desired Path'}, 'Location','northwest');
hold off;