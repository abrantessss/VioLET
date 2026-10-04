%% Position Error and Gamma Dynamics Plot
fig3 = figure('Units','centimeters','Position',[4 4 figureSize]);
te = t - t(1);

ax31 = subplot(2,1,1); hold(ax31,'on');
plot(ax31, te, vecnorm(pos' - pd', 2, 1), 'LineWidth', 2, 'LineJoin', 'chamfer');
plot(ax31, te, abs(pos(:,1)-pd(:,1)), 'LineWidth', 2, 'LineJoin', 'chamfer')
plot(ax31, te, abs(pos(:,2)-pd(:,2)), 'LineWidth', 2, 'LineJoin', 'chamfer')
plot(ax31, te, abs(pos(:,3)-pd(:,3)), 'LineWidth', 2, 'LineJoin', 'chamfer')
xlabel(ax31,'Time (s)'); ylabel(ax31,'Error (m)');
xlim(ax31, [0 te(end)]); ylim(ax31, [-1, inf]);
style_axes(ax31, fontsize);
legend(ax31, {'Norm Error','X-axis Error','Y-axis Error','Z-axis Error'}, 'Location','northeast');
hold(ax31,'off');

ax32 = subplot(2,1,2); hold(ax32,'on');
plot(ax32, te, vecnorm(v',2,1), 'LineWidth', 2,'LineJoin', 'chamfer');
plot(ax32, te, gamma_dot, 'LineWidth', 2,'LineJoin', 'chamfer');
plot(ax32, te, vd, 'LineWidth', 2,'LineJoin', 'chamfer');
xlabel(ax32,'Time (s)'); ylabel(ax32,'Velocity  (m/s)');
xlim(ax32, [0 te(end)]); ylim(ax32, [-0.5 max(vecnorm(v',2,1))+0.5]);
style_axes(ax32, fontsize);
legend(ax32, {'UAV Velocity', 'V. Target Velocity','Desired Velocity'}, 'Location','northeast');
hold(ax32,'off');

set(fig3,'Renderer','opengl');

%% Attitude Error Norm Plot (only when eR was read from the bag)
if exist('eR','var')
    fig4 = figure('Units','centimeters','Position',[8 4 figureSize]);
    ax4 = axes('Parent',fig4); hold(ax4,'on');
    plot(ax4, te, vecnorm(eR,2,2), 'LineWidth', 2, 'LineJoin', 'chamfer');
    xlabel(ax4,'Time (s)'); ylabel(ax4,'Attitude Error Norm ||e_R||');
    xlim(ax4, [0 te(end)]); ylim(ax4, [0, inf]);
    style_axes(ax4, fontsize);
    set(fig4,'Renderer','opengl');
    hold(ax4,'off');
end