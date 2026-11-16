"""Orient the four STEP-derived solids on the print bed, in millimetres."""
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
MESHES = HERE.parents[1] / "sim/source/meshes"
# Each rotation places a broad exterior face on the bed.
PARTS = {
    "motor-holder": (3, np.pi, (1, 0, 0)),
    "bottom": (4, 0, (1, 0, 0)),
    "rotary-arm": (5, -np.pi / 2, (0, 1, 0)),
    "pendulum-arm": (7, -np.pi / 2, (1, 0, 0)),
}

for name, (index, angle, axis) in PARTS.items():
    mesh = trimesh.load_mesh(MESHES / f"{index:02}.stl")
    mesh.apply_transform(trimesh.transformations.rotation_matrix(angle, axis))
    mesh.apply_translation(-mesh.bounds[0])
    if not mesh.is_volume or len(mesh.split(only_watertight=False)) != 1:
        raise ValueError(f"{name} is not one closed printable solid")
    mesh.export(HERE / f"{name}.stl")
    print(f"{name}: {mesh.extents.round(2)} mm")
