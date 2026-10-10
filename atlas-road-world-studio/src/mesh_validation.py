"""Fail export before packaging malformed physical or visual meshes."""
import numpy as np

def validate_meshes(meshes):
    for name,vertices,indices,material in meshes:
        if not vertices or not indices:raise ValueError('Empty mesh: '+name)
        if len(vertices)>65535:raise ValueError('KN5 vertex limit exceeded: '+name)
        values=np.asarray([tuple(v[0])+tuple(v[1])+tuple(v[2])+tuple(v[3]) for v in vertices],dtype=np.float32)
        if values.shape!=(len(vertices),11) or not np.isfinite(values).all():raise ValueError('Invalid mesh vertex: '+name)
        ix=np.asarray(indices)
        if len(ix)%3 or not np.issubdtype(ix.dtype,np.integer) or ix.min()<0 or ix.max()>=len(vertices):raise ValueError('Invalid triangle indices: '+name)
        # Round to serialized coordinates first, then compute in float64.
        # Float32 cross products can cancel to zero on valid thin triangles.
        tri=values[ix.reshape(-1,3),:3].astype(np.float64)
        normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
        if np.any(np.linalg.norm(normals,axis=1)<1e-9):raise ValueError('Degenerate triangle: '+name)
        if name.startswith(('1ROAD','1GRASS')) and np.any(normals[:,1]<=0):raise ValueError('Inverted physical triangle: '+name)
