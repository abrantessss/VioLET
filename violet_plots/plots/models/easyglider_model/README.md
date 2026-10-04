# EasyGlider - MATLAB 3-D model

Converted from the Gazebo model you supplied (`easyglider.sdf.jinja` +
`meshes/*.dae`). All link and visual poses from the SDF are baked in, so every
part shares one body frame: **+x forward, +y starboard, +z up, origin at the
SDF model origin**, units metres.

| file | what it is |
|---|---|
| `easyglider_mesh.json` | the geometry: 10 parts, ~79 k triangles, vertices in metres, 1-based faces |
| `easyglider_model.m` | loader -> struct with `.parts(k).V/.F/.color/.hinge` plus mass, inertia, S, MAC, AR |
| `plot_easyglider.m` | draws the parts as `patch` objects, returns a handle struct |
| `easyglider_set_controls.m` | deflects elevons / elevator / rudder, spins the prop, poses the aircraft in the world |
| `example_easyglider.m` | static render, control sweep, and a flown trajectory |

## Quick start

```matlab
M = easyglider_model();
figure; h = plot_easyglider(M);
axis equal; view(135,20); camlight; lighting gouraud

easyglider_set_controls(h, struct('aileron',20,'elevator',-12,'prop',45));
easyglider_set_controls(h, struct(), [10 0 5], [30 5 60]);   % pose in world
```

## Parts and hinges

| part | triangles | hinge |
|---|---|---|
| fuselage | 34 392 | - |
| rudder | 12 438 | x = -0.880, axis z |
| lid | 11 160 | - |
| left_wing / right_wing | 6 404 / 6 396 | - |
| nose | 3 784 | - |
| propeller | 3 065 | spin axis x at (0.115, 0, -0.085) |
| elevator | 1 290 | x = -0.880, axis y |
| left_elevon / right_elevon | 206 / 216 | x = -0.314, y = -+0.505, axis y |

Hinge positions recovered from the meshes match the SDF joint poses
(`-0.32 -+0.5 -0.045` for the elevons, `-0.88 0 -0.035` for elevator and
rudder), and travel is clamped to the SDF's +-30 deg (+-0.53 rad) limits.

## Aerodynamic constants carried along

`M.mass_kg` 1.48, `M.inertia_kgm2` diag(0.197563, 0.1458929, 0.1477),
`M.wing_area_m2` 0.416, `M.mac_m` 0.1541, `M.aspect_ratio` 7.788,
`M.span_m` 1.774, `M.length_m` 1.13 - so the same struct can feed a
6-DOF simulation and its animation.

## Performance note

~79 k triangles renders interactively in MATLAB. If you animate many frames
on a slow machine, drop `lid` and `fuselage` detail by plotting a subset of
`M.parts`, or call `reducepatch(h.patches(k), 0.3)` after `plot_easyglider`.

## Not included

`left_flap.dae`, `right_flap.dae` and `iris_prop_cw.dae` exist in the mesh
folder but are not referenced by the SDF, so they are left out. Say the word
and I'll add them.
