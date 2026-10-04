%% EXAMPLE_SHUTTLE  Plot and animate the shuttle quadrotor model.
% Run this file from the folder that holds shuttle_model.m, the mesh_*.csv
% hull files and the two iris_prop_*.csv files.

M = shuttle_model();                 % real shuttle.dae hull + real props
fprintf('%s: %.4f kg, arm %.3f m, prop radius %.4f m, %d parts, %d triangles\n', ...
    M.name, M.mass, M.arm, M.prop_radius, numel(M.parts), M.n_tri);

%% 1 - static three-quarter view
figure('Color','w','Name','shuttle - static');
h = shuttle_plot(M);
set(gca,'Color','w');
view(135,20); axis tight; axis equal
title('shuttle quadrotor (body frame)');

%% 2 - orthographic triptych
figure('Color','w','Name','shuttle - views');
vs = {[0 90] 'top'; [0 0] 'front'; [90 0] 'side'};
for k = 1:3
    subplot(1,3,k);
    shuttle_plot(shuttle_model());
    view(vs{k,1}); title(vs{k,2}); axis equal; axis tight
end

%% 3 - spin the rotors and fly a lissajous trajectory
% same real geometry as figures 1-2; flat lighting keeps 97k triangles fast
Ma = M;
figure('Color','w','Name','shuttle - animation');
h = shuttle_plot(Ma);
set(h.patches,'FaceLighting','flat');
hold on
t   = linspace(0,10,300);
pos = [1.6*sin(0.6*t)' 1.2*sin(0.9*t)' (1 + 0.25*sin(0.4*t))'];
plot3(pos(:,1),pos(:,2),pos(:,3),':','Color',[0.6 0.6 0.65]);
trail = animatedline('Color',[0 0.6 0.65],'LineWidth',1.2);
axis([-2.2 2.2 -1.8 1.8 0 2]); axis manual; view(140,18)
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');

w  = 0.35*Ma.max_rot_vel;                 % rotor speed [rad/s]
dt = mean(diff(t));
for k = 1:numel(t)
    % attitude roughly aligned with the acceleration (illustrative only)
    if k > 2
        acc = (pos(k,:) - 2*pos(k-1,:) + pos(k-2,:))/dt^2;
    else
        acc = [0 0 0];
    end
    roll  = -acc(2)/9.81;  pitch = acc(1)/9.81;  yaw = 0.4*t(k);
    ang = Ma.spin * w * t(k);             % per-rotor angle, respects cw/ccw
    shuttle_setpose(h,pos(k,:),[roll pitch yaw],ang);
    addpoints(trail,pos(k,1),pos(k,2),pos(k,3));
    drawnow limitrate
end

%% 4 - export the whole airframe as one triangulation (optional)
% V = []; F = [];
% for k = 1:numel(M.parts)
%     F = [F; M.parts(k).F + size(V,1)];
%     V = [V; M.parts(k).V];
% end
% stlwrite(triangulation(F,V),'shuttle.stl');
