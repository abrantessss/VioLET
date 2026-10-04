function M = shuttle_model(varargin)
%SHUTTLE_MODEL  Plottable geometry of the "shuttle" quadrotor (from shuttle.sdf.jinja).
%
%   M = SHUTTLE_MODEL() returns a struct with the airframe geometry, rotor
%   layout and mass properties taken straight out of the SDF model:
%
%     mass            3.4207 kg
%     inertia         diag([0.0216667 0.0216667 0.04]) kg m^2, com at z = 0.24 m
%     rotor_0  ( 0.248, -0.248, 0.36)  ccw
%     rotor_1  (-0.248,  0.248, 0.36)  ccw
%     rotor_2  ( 0.248,  0.248, 0.36)  cw
%     rotor_3  (-0.248, -0.248, 0.36)  cw
%     airframe        shuttle.dae  (96543 triangles, 3 material groups)
%     propeller       iris_prop_cw.dae (EPP1045, 10 in) scaled 1.2
%
%   Frame: SDF/Gazebo body frame -- x forward, y left, z up, metres. The DAE
%   meshes are y-up, so they are rotated +pi/2 about x on import (the same
%   1.571 rad visual pose the SDF applies).
%
%   Name-value options
%     'PropScale'    1.2     propeller scale, matches <scale> in the SDF
%     'MeshDir'      ''      folder with the mesh_*.csv / iris_prop_*.csv files
%                            (default: the folder this file lives in)
%     'Body'         'mesh'  'mesh' = real shuttle.dae hull
%                            'simple' = light parametric stand-in (fast animation)
%     'SimpleProps'  false   true -> analytic 2-blade props instead of the CAD mesh
%     'Facets'       24      facets used for cylinders in 'simple' mode
%
%   M.parts(k) has fields:
%     name, V (Nx3), F (Mx3), color (1x3), alpha, group
%   group is 'body' or 'rotor1'..'rotor4' so that the rotors can be spun
%   independently (see SHUTTLE_PLOT).
%
%   See also SHUTTLE_PLOT, SHUTTLE_SETPOSE, EXAMPLE_SHUTTLE.

opt = struct('PropScale',1.2,'MeshDir','','Body','mesh','SimpleProps',false,'Facets',24);
for k = 1:2:numel(varargin)
    opt.(varargin{k}) = varargin{k+1};
end
if isempty(opt.MeshDir)
    opt.MeshDir = fileparts(mfilename('fullpath'));
end

M = struct();
M.name       = 'shuttle';
M.mass       = 3.4207;
M.com        = [0 0 0.24];
M.inertia    = diag([0.02166666666666667 0.02166666666666667 0.04]);
M.arm        = 0.248;                       % rotor offset along x and y [m]
M.rotor_z    = 0.36;                        % rotor hub height [m]
M.rotors     = [ 0.248 -0.248 0.36;         % rotor_0
                -0.248  0.248 0.36;         % rotor_1
                 0.248  0.248 0.36;         % rotor_2
                -0.248 -0.248 0.36];        % rotor_3
M.spin       = [ 1  1 -1 -1];               % +1 = ccw about +z (SDF turningDirection)
M.max_rot_vel = 1400;                       % rad/s
M.motor_constant = 1.709716e-05;
M.moment_constant = 0.016;
M.imu_link   = [0 0 0.28];
M.gps_link   = [-0.12 -0.02 0.48];

% ---- palette (SDF / DAE materials) --------------------------------------
cBlue  = [0.0247 0.0178 0.8000];            % shuttle.dae "Blue"
cWhite = [0.9200 0.9200 0.9300];            % shuttle.dae "material_1"
cBlack = [0.0600 0.0600 0.0700];            % shuttle.dae "Black"
cProp  = [0 1 1];                           % SDF <diffuse>0 1 1 1</diffuse>

parts = struct('name',{},'V',{},'F',{},'color',{},'alpha',{},'group',{});
add = @(n,V,F,c,a,g) struct('name',n,'V',V,'F',F,'color',c,'alpha',a,'group',g);

% ---- airframe ------------------------------------------------------------
hull = {'Blue' cBlue; 'material_1' cWhite; 'Black' cBlack};
haveHull = strcmpi(opt.Body,'mesh');
if haveHull
    for k = 1:size(hull,1)
        [V,F,ok] = readMeshCsv(opt.MeshDir,hull{k,1});
        if ~ok, haveHull = false; break, end
        parts(end+1) = add(['hull_' hull{k,1}],V,F,hull{k,2},1,'body');
    end
end
if ~haveHull
    if strcmpi(opt.Body,'mesh')
        warning('shuttle_model:noHull', ...
            'mesh_*.csv not found in %s -- using the parametric body.',opt.MeshDir);
    end
    parts = struct('name',{},'V',{},'F',{},'color',{},'alpha',{},'group',{});
    parts = parametricBody(parts,add,M,opt);
end

% ---- propellers ----------------------------------------------------------
if opt.SimpleProps
    [Vp,Fp] = simpleProp(0.1295*opt.PropScale);
else
    [Vp,Fp] = propMesh(opt.MeshDir,opt.PropScale);
end
M.prop_radius = max(hypot(Vp(:,1),Vp(:,2)));
for i = 1:4
    r = M.rotors(i,:);
    V = Vp;
    if M.spin(i) < 0, V(:,2) = -V(:,2); end     % mirror cw mesh -> ccw
    V = V + repmat(r,size(V,1),1);
    parts(end+1) = add(sprintf('rotor_%d',i-1),V,Fp,cProp,1,sprintf('rotor%d',i));
end

M.parts = parts;
M.n_tri = sum(arrayfun(@(p) size(p.F,1), parts));
end

% =========================== local helpers ===============================
function [V,F,ok] = readMeshCsv(meshDir,tag)
fv = fullfile(meshDir,sprintf('mesh_%s_vertices.csv',tag));
ff = fullfile(meshDir,sprintf('mesh_%s_faces.csv',tag));
V = []; F = []; ok = isfile(fv) && isfile(ff);
if ~ok, return, end
V = readNum(fv);  F = readNum(ff);
end

function A = readNum(f)
if exist('readmatrix','file')
    A = readmatrix(f);
else
    A = dlmread(f,',');
end
end

function [V,F] = propMesh(meshDir,s)
fv = fullfile(meshDir,'iris_prop_vertices.csv');
ff = fullfile(meshDir,'iris_prop_faces.csv');
if ~isfile(fv) || ~isfile(ff)
    warning('shuttle_model:noProp', ...
        'Propeller CSVs not found in %s -- falling back to analytic blades.',meshDir);
    [V,F] = simpleProp(0.1295*s); return
end
V = readNum(fv)*s;  F = readNum(ff);
end

function parts = parametricBody(parts,add,M,opt)
cBody = [0.16 0.17 0.20]; cPlate = [0.24 0.26 0.30];
cArm  = [0.30 0.32 0.36];  cMotor = [0.55 0.57 0.60]; cLeg = [0.12 0.13 0.15];
[V,F] = boxMesh([0 0 0.24],[0.26 0.17 0.11]);   parts(end+1) = add('fuselage',V,F,cBody,1,'body');
[V,F] = boxMesh([0 0 0.305],[0.20 0.15 0.02]);  parts(end+1) = add('top_plate',V,F,cPlate,1,'body');
[V,F] = boxMesh([0.145 0 0.245],[0.06 0.10 0.07]); parts(end+1) = add('nose',V,F,cPlate,1,'body');
for i = 1:4
    r  = M.rotors(i,:);
    [V,F] = cylMesh([sign(r(1))*0.075 sign(r(2))*0.055 0.30],[r(1) r(2) 0.315],0.013,opt.Facets);
    parts(end+1) = add(sprintf('arm_%d',i-1),V,F,cArm,1,'body');
    [V,F] = cylMesh([r(1) r(2) 0.300],[r(1) r(2) 0.330],0.030,opt.Facets);
    parts(end+1) = add(sprintf('mount_%d',i-1),V,F,cPlate,1,'body');
    [V,F] = cylMesh([r(1) r(2) 0.315],[r(1) r(2) 0.352],0.021,opt.Facets);
    parts(end+1) = add(sprintf('motor_%d',i-1),V,F,cMotor,1,'body');
end
for sx = [-1 1]
    [V,F] = cylMesh([sx*0.10 -0.135 0],[sx*0.10 0.135 0],0.011,12);
    parts(end+1) = add(sprintf('skid_%+d',sx),V,F,cLeg,1,'body');
    for sy = [-1 1]
        [V,F] = cylMesh([sx*0.06 sy*0.06 0.20],[sx*0.10 sy*0.135 0],0.010,12);
        parts(end+1) = add(sprintf('leg_%+d%+d',sx,sy),V,F,cLeg,1,'body');
    end
end
end

function [V,F] = simpleProp(R)
t = linspace(0,1,12)';
c = 0.11*R*(0.55 + 0.9*sin(pi*t.^0.75));
x = R*t;
V = [x  c/2 0.004*sin(pi*t); x -c/2 -0.004*sin(pi*t)];
V = [V; -V(:,1) V(:,2) V(:,3)];
n = numel(t);
F = zeros(0,3);
for k = 1:n-1
    F = [F; k k+1 n+k; k+1 n+k+1 n+k];
end
F = [F; F+2*n];
[Vh,Fh] = cylMesh([0 0 -0.006],[0 0 0.006],0.09*R*2,16);
F = [F; Fh+size(V,1)];
V = [V; Vh];
end

function [V,F] = boxMesh(c,d)
h = d/2;
V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
V = V.*repmat(h,8,1) + repmat(c,8,1);
F = [1 2 3; 1 3 4; 5 6 7; 5 7 8; 1 2 6; 1 6 5; 2 3 7; 2 7 6; ...
     3 4 8; 3 8 7; 4 1 5; 4 5 8];
end

function [V,F] = cylMesh(p0,p1,r,n)
p0 = p0(:)'; p1 = p1(:)';
a = p1-p0; L = norm(a);
if L == 0, V = zeros(0,3); F = zeros(0,3); return, end
w = a/L;
u = cross(w,[0 0 1]);
if norm(u) < 1e-8, u = cross(w,[1 0 0]); end
u = u/norm(u);  v = cross(w,u);
th = linspace(0,2*pi,n+1); th(end) = [];
ring = r*(cos(th)'*u + sin(th)'*v);
V = [ring + repmat(p0,n,1); ring + repmat(p1,n,1); p0; p1];
F = zeros(0,3);
for k = 1:n
    k2 = mod(k,n)+1;
    F = [F; k k2 n+k2; k n+k2 n+k; ...
            2*n+1 k2 k; 2*n+2 n+k n+k2];
end
end
