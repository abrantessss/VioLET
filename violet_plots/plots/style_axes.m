function style_axes(ax, fontsize)
%STYLE_AXES  Common look for the thesis plots: visible grid, box, bold labels.
box(ax, 'on');
set(ax, 'XGrid', 'on', 'YGrid', 'on', 'ZGrid', 'on', 'GridLineStyle', '-', ...
    'GridColor', [0.15 0.15 0.15], 'GridAlpha', 0.35, 'LineWidth', 1.0, ...
    'Layer', 'bottom', 'FontSize', fontsize);
ax.XLabel.FontWeight = 'bold';
ax.YLabel.FontWeight = 'bold';
ax.ZLabel.FontWeight = 'bold';
% R2025a+ figures follow the desktop theme; force light so black lines show.
fig = ancestor(ax, 'figure');
set(fig, 'Color', 'w');
if ~isMATLABReleaseOlderThan('R2025a')
    theme(fig, 'light');
end
end
