%% Position Error and Gamma Dynamics Plot 
fig3 = figure('Units','centimeters','Position',[4 4 16 12]);

ax31 = subplot(2,1,1); hold(ax31,'on'); grid(ax31,'on');
plot(ax31, t, vecnorm(pos' - pd', 2, 1), 'LineWidth', 2, 'LineJoin', 'chamfer');
plot(ax31, t, abs(pos(:,1)-pd(:,1)), 'LineWidth', 2, 'LineJoin', 'chamfer')
plot(ax31, t, abs(pos(:,2)-pd(:,2)), 'LineWidth', 2, 'LineJoin', 'chamfer')
plot(ax31, t, abs(pos(:,3)-pd(:,3)), 'LineWidth', 2, 'LineJoin', 'chamfer')
xlabel(ax31,'Time (s)', 'FontWeight','bold'); ylabel(ax31,'Error (m)', 'FontWeight','bold');
ylim(ax31, [-1, inf]); %xlim(ax31, [0 15]);
legend(ax31, {'Norm Error','X-axis Error','Y-axis Error','Z-axis Error'}, 'Location','northeast');
hold off;

ax32 = subplot(2,1,2); hold(ax32,'on'); grid(ax32,'on');
plot(ax32, t, vecnorm(v',2,1), 'LineWidth', 2,'LineJoin', 'chamfer');
plot(ax32, t, gamma_dot, 'LineWidth', 2,'LineJoin', 'chamfer');
plot(ax32, t, vd, 'LineWidth', 2,'LineJoin', 'chamfer');
xlabel(ax32,'Time (s)', 'FontWeight','bold'); ylabel(ax32,'Velocity  (m/s)', 'FontWeight','bold');
ylim(ax32, [-0.5 max(vecnorm(v',2,1))+0.5]); %xlim(ax32, [0 15])
legend(ax32, {'UAV Velocity', 'V. Target Velocity','Desired Velocity'}, 'Location','best');

set(findall(fig3,'-property','FontSize'),'FontSize', fontsize); 
set(gcf,'Renderer','opengl');
hold off;