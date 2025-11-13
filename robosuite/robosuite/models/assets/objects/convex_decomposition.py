import trimesh
import numpy as np
import argparse
import os
parser = argparse.ArgumentParser()
parser.add_argument(
    "--mesh_path",
    type=str,
    default=None)
parser.add_argument(
    "--mesh_res",
    type=int,
    default=40000000,
    help="resolution of the decomposed mesh"
)
args = parser.parse_args()

# Load mesh
path_of_asset=args.mesh_path
mesh = trimesh.load(path_of_asset)

convex_pieces = trimesh.decomposition.convex_decomposition(
    mesh,
    maxConvexHulls=12,
    resolution=1000000  
)
mesh_folder = os.path.dirname(path_of_asset)
# Save each convex piece
for i, piece in enumerate(convex_pieces):
    if isinstance(piece, dict):
        convex_mesh = trimesh.Trimesh(vertices=piece['vertices'], faces=piece['faces'])
    else:
        convex_mesh = piece
    
    output_path = os.path.join(mesh_folder, f'cuboid_convex_{i}.stl')
    convex_mesh.export(output_path)
    print(f"Saved: {output_path}")

print(f"Decomposed into {len(convex_pieces)} convex parts")