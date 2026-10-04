%% EasyGlider 3-D model - example
% Loads the airframe converted from the Gazebo SDF model and shows three
% things: a static render, a control-surface sweep, and the aircraft flown
% along a trajectory.
%
% Run this file from the folder that contains easyglider_mesh.json.

clear; clc;
M = easyglider_model();

fprintf('EasyGlider  span %.3f m | length %.3f m | mass %.2f kg | S %.3f m^2\n', ...
        M.span_m, M.length_m, M.mass_kg, M.wing_area_m2);
fprintf('%d parts, %d triangles total\n', numel(M.parts), sum(arrayfun(@(p) size(p.F,1), M.parts)));

%% 1 - Static render
figure('Color', [0.09 0.10 0.12], 'Name', 'EasyGlider');
ax = axes('Color', 'none', 'XColor', [.6 .6 .6], 'YColor', [.6 .6 .6], 'ZColor', [.6 .6 .6]);
h = plot_easyglider(M, 'Parent', ax);
axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'off');
xlabel('x_b  [m]'); ylabel('y_b  [m]'); zlabel('z_b  [m]');
view(ax, 135, 22);
camlight('headlight'); camlight('left'); lighting gouraud; material dull;
title('EasyGlider - body frame', 'Color', 'w');

%% 2 - Control-surface sweep
% Aileron, elevator and rudder driven through their +-30 deg travel.
figure('Color', [0.09 0.10 0.12], 'Name', 'Control sweep');
ax2 = axes('Color', 'none'); axis(ax2, 'equal'); axis(ax2, 'off');
h2 = plot_easyglider(M, 'Parent', ax2);
view(ax2, 140, 18); camlight('headlight'); lighting gouraud;
xlim(ax2, [-1.2 0.3]); ylim(ax2, [-1 1]); zlim(ax2, [-0.4 0.5]);

t = linspace(0, 4*pi, 240);
for i = 1:numel(t)
    ctrl.aileron  = 30 * sin(t(i));
    ctrl.elevator = 25 * sin(t(i) + pi/3);
    ctrl.rudder   = 25 * sin(t(i) + 2*pi/3);
    ctrl.prop     = 12 * i;                 % windmilling propeller
    easyglider_set_controls(h2, ctrl);
    drawnow limitrate;
end

%% 3 - Fly a trajectory
% A climbing turn, with bank angle and control deflections driven by the
% path. This is the pattern to reuse for plotting simulation output.
figure('Color', [0.09 0.10 0.12], 'Name', 'Trajectory');
ax3 = axes('Color', 'none', 'XColor', [.5 .5 .5], 'YColor', [.5 .5 .5], 'ZColor', [.5 .5 .5]);
hold(ax3, 'on'); grid(ax3, 'on');

R = 30; V = 14; dt = 0.05; N = 400;
tt = (0:N-1) * dt;
psi = V/R * tt;
pos = [R*sin(psi(:)), R*(1-cos(psi(:))), 2 + 0.8*tt(:)];
roll = 22 * ones(N,1);
pitch = 4 * ones(N,1);

plot3(ax3, pos(:,1), pos(:,2), pos(:,3), '-', 'Color', [0.45 0.72 0.95], 'LineWidth', 1.2);
h3 = plot_easyglider(M, 'Parent', ax3, 'Scale', 6);   % scaled up to stay visible
axis(ax3, 'equal'); view(ax3, -35, 18);
camlight('headlight'); lighting gouraud;
xlabel('North [m]'); ylabel('East [m]'); zlabel('Up [m]');
title('Climbing turn', 'Color', 'w');

for i = 1:4:N
    ctrl = struct('aileron', 8, 'elevator', -3, 'rudder', 4, 'prop', 25*i);
    easyglider_set_controls(h3, ctrl, pos(i,:), [roll(i) pitch(i) rad2deg(psi(i))]);
    drawnow limitrate;
end
