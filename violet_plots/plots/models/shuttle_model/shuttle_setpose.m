function shuttle_setpose(h,pos,rpy,propAngles)
%SHUTTLE_SETPOSE  Move/orient a plotted shuttle and optionally spin its props.
%
%   SHUTTLE_SETPOSE(h,pos,rpy) places the vehicle at pos = [x y z] with
%   attitude rpy = [roll pitch yaw] in radians (ZYX / yaw-pitch-roll).
%   SHUTTLE_SETPOSE(h,pos,rpy,propAngles) also sets the four rotor angles
%   (1x4, radians); pass a scalar to apply it to all four.
%
%   See also SHUTTLE_PLOT.

if nargin < 3 || isempty(rpy), rpy = [0 0 0]; end
set(h.body,'Matrix', makehgtform('translate',pos) * ...
    makehgtform('zrotate',rpy(3)) * makehgtform('yrotate',rpy(2)) * ...
    makehgtform('xrotate',rpy(1)));

if nargin > 3 && ~isempty(propAngles)
    if isscalar(propAngles), propAngles = repmat(propAngles,1,4); end
    R = [0.248 -0.248 0.36; -0.248 0.248 0.36; 0.248 0.248 0.36; -0.248 -0.248 0.36];
    for i = 1:4
        set(h.rotor(i),'Matrix', makehgtform('translate',R(i,:)) * ...
            makehgtform('zrotate',propAngles(i)));
    end
end
end
