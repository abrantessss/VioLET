%% XY Plane Path Plot
fig2 = figure('Units','centimeters','Position',[6 6 16 12]);
ax2  = axes('Parent',fig2); hold(ax2,'on'); grid(ax2,'on');
axis(ax2,'equal'); set(ax2, 'YDir','reverse'); set(ax2, 'ZDir','reverse');
xlabel(ax2,'North [x] (m)','FontWeight','bold');
ylabel(ax2,'East [y] (m)','FontWeight','bold');

% Paths
hpath1 = plot(ax2, pos(:,1),  pos(:,2),  '-', 'LineWidth', 2, 'Color', "#ffa500");
hpd1 = plot(ax2, pd(:,1), pd(:,2), '--', 'LineWidth', 2, 'Color', "#1f77b4");
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
legend(ax2, [hpath1, hpd1], {'Vehicle Path', 'Desired Path'}, 'Location','northeast');
hold off;
