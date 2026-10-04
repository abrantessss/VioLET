function draw_vehicle(ax, vehicle, p, R, L, flat)
%DRAW_VEHICLE  Draw the shuttle or the easyglider with its body axes (NED plot).
%
%   draw_vehicle(ax, vehicle, p, R, L, flat)
%     vehicle.M       struct from shuttle_model() or easyglider_model()
%     vehicle.toFRD   3x3 map from the model frame to forward-right-down
%     vehicle.centre  1x3 model point placed at the vehicle position
%     vehicle.scale   drawing scale
%     p     1x3 position in NED            R  3x3 body (FRD) -> NED rotation
%     L     length of the body axes (m)
%     flat  false: 3-D view, draws x (red), y (green), z (blue)
%           true : top view, vehicle in the z = 0 plane, draws x and y only

p = p(:)';
if flat, p(3) = 0; end
T = R * vehicle.toFRD;
zTop = p(3);
for k = 1:numel(vehicle.M.parts)
    part = vehicle.M.parts(k);
    V = (T * ((part.V - vehicle.centre) * vehicle.scale)')' + p;
    zTop = min(zTop, min(V(:,3)));          % NED: smallest z is the highest point
    patch('Parent', ax, 'Faces', part.F, 'Vertices', V, ...
        'FaceColor', part.color, 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.45, ...
        'DiffuseStrength', 0.75, 'SpecularStrength', 0.25, ...
        'BackFaceLighting', 'reverselit', 'HandleVisibility', 'off');
end

% Body axes: x forward (red), y right (green), z down (blue).
colors = {[0.9 0 0], [0 0.8 0], [0 0 0.9]};
o = p;
if flat
    o(3) = zTop - 0.01 * L;                 % just above the hull, towards the camera
    nAxes = 2;
else
    nAxes = 3;
end
for k = 1:nAxes
    tip = o + L * R(:,k)';
    plot3(ax, [o(1) tip(1)], [o(2) tip(2)], [o(3) tip(3)], ...
        'Color', colors{k}, 'LineWidth', 2.5, 'HandleVisibility', 'off');
end
end
