function easyglider_set_controls(h, ctrl, position, attitude)
%EASYGLIDER_SET_CONTROLS  Deflect surfaces and pose the EasyGlider.
%
%   EASYGLIDER_SET_CONTROLS(h, ctrl) updates the patches created by
%   PLOT_EASYGLIDER. ctrl is a struct, all angles in DEGREES, all optional:
%
%     ctrl.aileron   differential elevon: +ve = right roll command
%     ctrl.elevator  +ve = trailing edge up  = nose-up pitch command
%     ctrl.rudder    +ve = trailing edge left = nose-left yaw command
%     ctrl.flap      symmetric elevon deflection (both trailing edges down)
%     ctrl.prop      propeller rotation angle about the thrust axis
%
%   Elevon / elevator / rudder travel is clamped to the +-30 deg limit
%   declared in the SDF joints.
%
%   EASYGLIDER_SET_CONTROLS(h, ctrl, position, attitude) additionally places
%   the aircraft in the world: position = [x y z] (m), attitude =
%   [roll pitch yaw] (deg), applied as R = Rz(yaw)*Ry(pitch)*Rx(roll).
%
%   See also EASYGLIDER_MODEL, PLOT_EASYGLIDER.

if nargin < 2 || isempty(ctrl),      ctrl = struct();      end
if nargin < 3 || isempty(position),  position = [0 0 0];   end
if nargin < 4 || isempty(attitude),  attitude = [0 0 0];   end

g = @(f) fieldOr(ctrl, f, 0);
da = g('aileron'); de = g('elevator'); dr = g('rudder');
df = g('flap');    dp = g('prop');

R = rotz(attitude(3)) * roty(attitude(2)) * rotx(attitude(1));
s = h.scale;
M = h.model;

for k = 1:numel(M.parts)
    p = M.parts(k);
    V = p.V;

    switch p.name
        case 'left_elevon',  V = hingeRotate(V, p.hinge,  clampSurf(-da + df, p));
        case 'right_elevon', V = hingeRotate(V, p.hinge,  clampSurf( da + df, p));
        case 'elevator',     V = hingeRotate(V, p.hinge,  clampSurf( de, p));
        case 'rudder',       V = hingeRotate(V, p.hinge,  clampSurf( dr, p));
        case 'propeller'
            if ~isempty(p.spin)
                V = axisRotate(V, p.spin.origin, p.spin.axis, dp);
            end
    end

    Vw = (R * (V * s)')' + position;
    set(h.patches(k), 'Vertices', Vw);
end
end

% ---------------------------------------------------------------- helpers
function v = fieldOr(s, f, d)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function a = clampSurf(a, p)
lim = 30;
if ~isempty(p.hinge) && isfield(p.hinge, 'limit_deg'), lim = p.hinge.limit_deg; end
a = max(-lim, min(lim, a));
end

function V = hingeRotate(V, hinge, angDeg)
if isempty(hinge) || angDeg == 0, return; end
V = axisRotate(V, hinge.origin, hinge.axis, angDeg);
end

function V = axisRotate(V, origin, axis, angDeg)
if angDeg == 0, return; end
u = axis(:) / norm(axis);
t = deg2rad(angDeg);
K = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
R = eye(3) + sin(t)*K + (1-cos(t))*(K*K);   % Rodrigues
V = ((R * (V - origin)')') + origin;
end

function R = rotx(d), t = deg2rad(d); R = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)]; end
function R = roty(d), t = deg2rad(d); R = [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)]; end
function R = rotz(d), t = deg2rad(d); R = [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1]; end
