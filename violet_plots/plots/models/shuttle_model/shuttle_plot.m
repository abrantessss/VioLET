function h = shuttle_plot(M,varargin)
%SHUTTLE_PLOT  Draw the shuttle quadrotor and return handles for animation.
%
%   h = SHUTTLE_PLOT(M) plots the model returned by SHUTTLE_MODEL into gca.
%
%   Name-value options
%     'Parent'    gca      axes (or hgtransform) to draw into
%     'EdgeColor' 'none'   patch edge colour
%     'PropAlpha' 0.9      propeller face alpha
%
%   Returns
%     h.body      hgtransform for the whole vehicle -- set its Matrix to move it
%     h.rotor(4)  hgtransform per rotor, child of h.body, for prop spin
%     h.patches   all patch handles
%
%   Example
%     M = shuttle_model();  h = shuttle_plot(M);
%     shuttle_setpose(h,[0 0 1],[0 0.2 0.6]);
%
%   See also SHUTTLE_MODEL, SHUTTLE_SETPOSE.

opt = struct('Parent',[],'EdgeColor','none','PropAlpha',0.9);
for k = 1:2:numel(varargin), opt.(varargin{k}) = varargin{k+1}; end
if isempty(opt.Parent), opt.Parent = gca; end

h.body = hgtransform('Parent',opt.Parent);
for i = 1:4
    h.rotor(i) = hgtransform('Parent',h.body, ...
        'Matrix',makehgtform('translate',M.rotors(i,:)));
end

h.patches = gobjects(0);
for k = 1:numel(M.parts)
    P = M.parts(k);
    if strcmp(P.group,'body')
        parent = h.body;  V = P.V;  fa = P.alpha;
    else
        i = str2double(P.group(6:end));
        parent = h.rotor(i);
        V = P.V - repmat(M.rotors(i,:),size(P.V,1),1);   % re-centre on the hub
        fa = opt.PropAlpha;
    end
    h.patches(end+1) = patch('Parent',parent,'Faces',P.F,'Vertices',V, ...
        'FaceColor',P.color,'EdgeColor',opt.EdgeColor,'FaceAlpha',fa, ...
        'FaceLighting','gouraud','SpecularStrength',0.35,'AmbientStrength',0.4, ...
        'DiffuseStrength',0.8,'BackFaceLighting','reverselit','Tag',P.name);
end

ax = ancestor(h.body,'axes');
axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
if isempty(findobj(ax,'Type','light'))
    view(ax,135,22);
    light(ax,'Position',[ 1  1  2],'Style','infinite');
    light(ax,'Position',[-1 -0.5 0.5],'Style','infinite','Color',[0.35 0.4 0.5]);
end
end
