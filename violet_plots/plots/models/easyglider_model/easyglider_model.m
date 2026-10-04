function M = easyglider_model(jsonFile)
%EASYGLIDER_MODEL  Load the EasyGlider airframe as MATLAB patch geometry.
%
%   M = EASYGLIDER_MODEL() loads easyglider_mesh.json from the same folder
%   as this file. M is a struct with fields:
%
%     M.parts     1xN struct array, one entry per airframe part:
%                   .name    'fuselage','nose','left_wing','right_wing',
%                            'lid','left_elevon','right_elevon',
%                            'elevator','rudder','propeller'
%                   .V       Nv x 3 vertex matrix, metres, body frame
%                   .F       Nf x 3 triangle faces, 1-based (patch-ready)
%                   .color   1 x 3 RGB
%                   .hinge   [] or struct('origin',[x y z],'axis',[x y z],
%                            'limit_deg',30)  - control-surface hinge line
%                   .spin    [] or struct('origin',...,'axis',...)  - prop
%     M.mass_kg, M.inertia_kgm2, M.wing_area_m2, M.mac_m, M.aspect_ratio,
%     M.span_m, M.length_m, M.cg_body
%
%   Body frame: +x forward (nose), +z up, +y out the starboard wing.
%   Geometry is converted from the Gazebo SDF model (meshes/*.dae) with the
%   link/visual poses of easyglider.sdf.jinja already baked in, so every
%   part is expressed in one common body frame.
%
%   See also PLOT_EASYGLIDER, EASYGLIDER_SET_CONTROLS.

if nargin < 1 || isempty(jsonFile)
    jsonFile = fullfile(fileparts(mfilename('fullpath')), 'easyglider_mesh.json');
end
raw = jsondecode(fileread(jsonFile));

p = raw.parts;
if ~iscell(p), p = num2cell(p); end

parts = struct('name', {}, 'V', {}, 'F', {}, 'color', {}, 'hinge', {}, 'spin', {});
for k = 1:numel(p)
    s = p{k};
    q.name  = s.name;
    q.V     = double(s.V);
    q.F     = double(s.F);
    q.color = hex2rgb(s.color);
    q.hinge = [];
    q.spin  = [];
    if isfield(s, 'hinge')
        q.hinge = struct('origin', double(s.hinge.origin(:)'), ...
                         'axis',   double(s.hinge.axis(:)'), ...
                         'limit_deg', double(s.hinge.limit_deg));
    end
    if isfield(s, 'spin')
        q.spin = struct('origin', double(s.spin.origin(:)'), ...
                        'axis',   double(s.spin.axis(:)'));
    end
    parts(k) = q; %#ok<AGROW>
end

M = rmfield(raw, 'parts');
M.parts = parts;
end

function rgb = hex2rgb(h)
h = char(h);
if h(1) == '#', h(1) = []; end
rgb = [hex2dec(h(1:2)) hex2dec(h(3:4)) hex2dec(h(5:6))] / 255;
end
