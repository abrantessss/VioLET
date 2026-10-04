function h = plot_easyglider(M, varargin)
%PLOT_EASYGLIDER  Render the EasyGlider in 3-D as MATLAB patch objects.
%
%   h = PLOT_EASYGLIDER(M) draws the model returned by EASYGLIDER_MODEL
%   into the current axes and returns a handle struct for animation.
%
%   h = PLOT_EASYGLIDER(M, 'Name', value, ...) options:
%     'Parent'    axes handle                            (default: gca)
%     'Position'  1x3 world position of the body origin  (default: [0 0 0])
%     'Attitude'  [roll pitch yaw] in degrees            (default: [0 0 0])
%     'Scale'     uniform scale factor                   (default: 1)
%     'Controls'  struct with fields aileron, elevator, rudder, prop
%                 (degrees; see EASYGLIDER_SET_CONTROLS) (default: zeros)
%     'EdgeColor' 'none' or a colour                     (default: 'none')
%     'FaceAlpha' 0..1                                   (default: 1)
%
%   The returned struct h is passed straight to EASYGLIDER_SET_CONTROLS to
%   move control surfaces / spin the propeller / fly the aircraft along a
%   trajectory without rebuilding the patches.
%
%   Example:
%     M = easyglider_model();
%     figure; plot_easyglider(M); axis equal; view(135, 20); camlight
%
%   See also EASYGLIDER_MODEL, EASYGLIDER_SET_CONTROLS.

ip = inputParser;
ip.addParameter('Parent', []);
ip.addParameter('Position', [0 0 0]);
ip.addParameter('Attitude', [0 0 0]);
ip.addParameter('Scale', 1);
ip.addParameter('Controls', struct());
ip.addParameter('EdgeColor', 'none');
ip.addParameter('FaceAlpha', 1);
ip.parse(varargin{:});
o = ip.Results;

ax = o.Parent;
if isempty(ax), ax = gca; end
washold = ishold(ax);
hold(ax, 'on');

h.axes  = ax;
h.model = M;
h.scale = o.Scale;
h.patches = gobjects(1, numel(M.parts));

for k = 1:numel(M.parts)
    p = M.parts(k);
    h.patches(k) = patch('Parent', ax, ...
        'Faces', p.F, 'Vertices', p.V * o.Scale, ...
        'FaceColor', p.color, 'EdgeColor', o.EdgeColor, ...
        'FaceAlpha', o.FaceAlpha, 'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.42, 'DiffuseStrength', 0.72, ...
        'SpecularStrength', 0.28, 'SpecularExponent', 18, ...
        'BackFaceLighting', 'reverselit', 'Tag', p.name);
end

axis(ax, 'equal');
if ~washold, hold(ax, 'off'); end

easyglider_set_controls(h, o.Controls, o.Position, o.Attitude);
end
