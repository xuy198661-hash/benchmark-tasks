"""URDF convex collision hulls and bounded GJK distance; no simulator imports."""
from functools import lru_cache
from itertools import combinations, product
import struct
import xml.etree.ElementTree as ET
import numpy as np
from yam_arena import YAM_URDF
from yam_arena.urdf import rpy_matrix, quat_xyzw_to_matrix

CORNERS = np.array(list(product((-1., 1.), repeat=3)))

@lru_cache(maxsize=1)
def collision_hulls():
    result = []
    for link in ET.parse(YAM_URDF).getroot().findall('link'):
        for collision in link.findall('collision'):
            origin = collision.find('origin')
            rotation, xyz = np.eye(3), np.zeros(3)
            if origin is not None:
                rotation = rpy_matrix(np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
                xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                raise ValueError('unexpected non-mesh URDF collision geometry')
            path = YAM_URDF.parent / mesh.get('filename')
            data = path.read_bytes()
            count = struct.unpack('<I', data[80:84])[0]
            dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
            tri = np.frombuffer(data, dtype=dtype, offset=84, count=count)['v'].astype(float)
            vertices = np.unique(tri.reshape(-1, 3), axis=0)
            vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            result.append((link.get('name'), path.name, vertices @ rotation.T + xyz))
    return tuple(result)


def _closest_simplex(points):
    # Exhaust the faces of a <=5 point simplex; constrained closest point to zero.
    best = None
    for count in range(1, min(4, len(points))+1):
        for ids in combinations(range(len(points)), count):
            p = points[list(ids)]
            if count == 1:
                weights = np.ones(1)
            else:
                edges = (p[1:]-p[0]).T
                coeff = np.linalg.lstsq(edges, -p[0], rcond=None)[0]
                weights = np.r_[1-coeff.sum(), coeff]
            if weights.min() < -1e-10:
                continue
            weights = np.maximum(weights, 0); weights /= weights.sum()
            point = weights @ p
            squared = float(point @ point)
            if best is None or squared < best[0]:
                best = squared, np.asarray(ids), weights, point
    return best[1:]


def convex_distance(a, b, tolerance=1e-9, max_iterations=64):
    """Convex hull Euclidean distance with witnesses and a certified lower bound.

    An unconverged result is an interval, never advertised as an exact gap.
    At contact/intersection this reports zero, not a penetration depth.
    """
    a, b = np.asarray(a), np.asarray(b)
    wa, wb = [a[0]], [b[0]]
    lower = 0.
    for iteration in range(max_iterations):
        aa, bb = np.asarray(wa), np.asarray(wb)
        ids, weights, v = _closest_simplex(aa-bb)
        aa, bb = aa[ids], bb[ids]
        pa, pb = weights @ aa, weights @ bb
        upper = float(np.linalg.norm(v))
        if upper <= tolerance:
            return dict(distance_m=0., lower_m=0., upper_m=upper, converged=True,
                        witness_a_m=pa.tolist(), witness_b_m=pb.tolist(), iterations=iteration+1)
        sa, sb = a[np.argmin(a @ v)], b[np.argmax(b @ v)]
        lower = max(lower, float(v @ (sa-sb))/upper, 0.)
        if upper-lower <= tolerance:
            return dict(distance_m=upper, lower_m=lower, upper_m=upper, converged=True,
                        witness_a_m=pa.tolist(), witness_b_m=pb.tolist(), iterations=iteration+1)
        wa, wb = [*aa, sa], [*bb, sb]
    return dict(distance_m=None, lower_m=lower, upper_m=upper, converged=False,
                witness_a_m=pa.tolist(), witness_b_m=pb.tolist(), iterations=max_iterations)


def nearest_fixture(world_hulls, fixtures):
    """Broadphase lower bounds prune pairs, then GJK resolves the minimum."""
    low = np.array([v.min(0) for _, _, v in world_hulls])
    high = np.array([v.max(0) for _, _, v in world_hulls])
    flo = np.array([v.min(0) for _, v in fixtures])
    fhi = np.array([v.max(0) for _, v in fixtures])
    delta = np.maximum(np.maximum(flo[None]-high[:, None], low[:, None]-fhi[None]), 0)
    bounds = np.linalg.norm(delta, axis=2)
    pairs = []
    for flat in np.argsort(bounds, axis=None):
        i, j = np.unravel_index(flat, bounds.shape)
        link, mesh, vertices = world_hulls[i]
        name, box = fixtures[j]
        pairs.append((float(bounds[i, j]), link, mesh, name, vertices, box))
    best, global_lower = None, float('inf')
    for bound, link, mesh, name, vertices, box in pairs:
        if best is not None and bound >= best['upper_m']:
            break
        distance = convex_distance(vertices, box)
        global_lower = min(global_lower, distance['lower_m'])
        if best is None or distance['upper_m'] < best['upper_m']:
            best = dict(distance, link=link, mesh=mesh, fixture=name)
    if best is not None:
        best['lower_m'] = global_lower
        best['converged'] = best['upper_m']-global_lower <= 1e-9
        if not best['converged']:
            best['distance_m'] = None
        best['method'] = 'URDF convex collision hulls / GJK distance interval'
    return best
