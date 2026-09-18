"""Fit the board grid from the detected hole centres and emit the C++ constants.

The detection finds 41 of the 42 bores cleanly -- one column has three of its
holes joined into a single connected component by the modelling, so that group
is rejected as the wrong shape. 41 points over-determine a 7 x 6 regular grid, so
the grid is fitted from what was found and the missing cell falls out of the fit
rather than being measured.

Output is an origin and a pitch in the frame mesh's own local space. The runtime
multiplies these through the frame node's world transform, so the board can still
be moved, rotated and rescaled in the editor.
"""
import json
import os
import struct
import math

GLTF = '/c/Users/NoSig/Documents/connect4/external/assets/gltf/MainFrame/MainFrame.gltf'

COMP = {5120: ('b', 1), 5121: ('B', 1), 5122: ('h', 2),
        5123: ('H', 2), 5125: ('I', 4), 5126: ('f', 4)}
NCOMP = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}


def read_accessor(g, buffers, idx):
    acc = g['accessors'][idx]
    bv = g['bufferViews'][acc['bufferView']]
    fmt, size = COMP[acc['componentType']]
    n = NCOMP[acc['type']]
    data = buffers[bv.get('buffer', 0)]
    start = bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
    stride = bv.get('byteStride') or (size * n)
    out = []
    for i in range(acc['count']):
        vals = struct.unpack_from('<' + fmt * n, data, start + i * stride)
        out.append(vals[0] if n == 1 else vals)
    return out


class UF:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, a):
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


def cluster1d(vals, tol):
    """Group near-equal values and return their means, in order."""
    vals = sorted(vals)
    groups = [[vals[0]]]
    for v in vals[1:]:
        if v - groups[-1][-1] <= tol:
            groups[-1].append(v)
        else:
            groups.append([v])
    return [sum(gp) / len(gp) for gp in groups]


def main():
    base = os.path.dirname(GLTF)
    g = json.load(open(GLTF))
    buffers = [open(os.path.join(base, b['uri']), 'rb').read() for b in g['buffers']]

    prim = g['meshes'][0]['primitives'][0]
    pos = read_accessor(g, buffers, prim['attributes']['POSITION'])
    nrm = read_accessor(g, buffers, prim['attributes']['NORMAL'])
    idx = read_accessor(g, buffers, prim['indices'])

    n = len(pos)
    lo = [min(p[i] for p in pos) for i in range(3)]
    hi = [max(p[i] for p in pos) for i in range(3)]
    size = [hi[i] - lo[i] for i in range(3)]
    axis = size.index(min(size))
    pu, pv = [i for i in range(3) if i != axis]

    isWall = [abs(nrm[i][axis]) < 0.35 for i in range(n)]

    quant = max(size) * 1e-6
    weld = {}
    rep = [0] * n
    for i in range(n):
        key = tuple(round(pos[i][k] / quant) for k in range(3))
        if key not in weld:
            weld[key] = i
        rep[i] = weld[key]

    uf = UF(n)
    for t in range(0, len(idx), 3):
        a, b, c = idx[t], idx[t + 1], idx[t + 2]
        if isWall[a] and isWall[b] and isWall[c]:
            uf.union(rep[a], rep[b])
            uf.union(rep[a], rep[c])

    groups = {}
    for i in range(n):
        if isWall[i]:
            groups.setdefault(uf.find(rep[i]), []).append(i)

    pitchGuess = size[pu] / 7.0
    holes = []
    for verts in groups.values():
        if len(verts) < 8:
            continue
        us = [pos[i][pu] for i in verts]
        vs = [pos[i][pv] for i in verts]
        w, h = max(us) - min(us), max(vs) - min(vs)
        if w <= 1e-6 or h <= 1e-6:
            continue
        if max(w, h) < pitchGuess * 1.25 and max(w, h) / min(w, h) < 1.7:
            holes.append((sum(us) / len(us), sum(vs) / len(vs)))

    print('detected %d holes' % len(holes))

    cols = cluster1d([h[0] for h in holes], pitchGuess * 0.4)
    rows = cluster1d([h[1] for h in holes], pitchGuess * 0.4)
    print('distinct columns: %d -> %s' % (len(cols), [round(c, 2) for c in cols]))
    print('distinct rows:    %d -> %s' % (len(rows), [round(r, 2) for r in rows]))

    if len(cols) != 7 or len(rows) != 6:
        print('=> grid fit failed')
        return

    # Least-squares fit of a regular grid to the detected line positions, so a single
    # slightly-off hole cannot drag the whole layout.
    def fit(vals):
        k = len(vals)
        xs = list(range(k))
        mx = sum(xs) / k
        my = sum(vals) / k
        num = sum((xs[i] - mx) * (vals[i] - my) for i in range(k))
        den = sum((xs[i] - mx) ** 2 for i in range(k))
        pitch = num / den
        origin = my - pitch * mx
        resid = max(abs(vals[i] - (origin + pitch * i)) for i in range(k))
        return origin, pitch, resid

    u0, du, ru = fit(cols)
    v0, dv, rv = fit(rows)

    print()
    print('column origin %.3f  pitch %.3f  max residual %.3f' % (u0, du, ru))
    print('row    origin %.3f  pitch %.3f  max residual %.3f' % (v0, dv, rv))

    # Which cells were actually seen, so the missing one is stated rather than assumed
    seen = set()
    for hu, hv in holes:
        ci = round((hu - u0) / du)
        ri = round((hv - v0) / dv)
        seen.add((ci, ri))
    missing = [(c, r) for r in range(6) for c in range(7) if (c, r) not in seen]
    print('cells not directly detected: %s' % (missing if missing else 'none'))

    zc = (lo[axis] + hi[axis]) / 2
    print('board mid-thickness (%s) = %.3f' % ('XYZ'[axis], zc))
    print('board bounds min %s max %s' % ([round(v, 2) for v in lo], [round(v, 2) for v in hi]))

    print()
    print('--- C++ ---')
    print('const float kGridOriginX = %.4ff;   // centre of column 0' % u0)
    print('const float kGridOriginY = %.4ff;   // centre of row 0 (bottom)' % v0)
    print('const float kGridPitchX  = %.4ff;' % du)
    print('const float kGridPitchY  = %.4ff;' % dv)
    print('const float kGridPlaneZ  = %.4ff;   // mid-thickness of the panel' % zc)
    print('const float kFrameTopY   = %.4ff;' % hi[pv])


main()


def emit_fractions():
    """Re-run the fit and print the grid as fractions of the mesh bounding box."""
    import io
    lo = [-416.5601, 135.8890, -65.4834]
    hi = [416.5601, 806.8204, 2.4589]
    cols = [-330.09, -216.94, -109.49, -2.15, 106.81, 215.61, 327.40]
    rows = [198.67, 308.44, 418.81, 525.87, 633.89, 740.42]
    sx = hi[0] - lo[0]
    sy = hi[1] - lo[1]
    sz = hi[2] - lo[2]
    print()
    print('--- as fractions of the mesh bounds ---')
    print('const float kColFrac[C4::kCols] = {')
    print('    ' + ', '.join('%.6ff' % ((c - lo[0]) / sx) for c in cols))
    print('};')
    print('const float kRowFrac[C4::kRows] = {')
    print('    ' + ', '.join('%.6ff' % ((r - lo[1]) / sy) for r in rows))
    print('};')
    print('const float kPlaneFrac = %.6ff;' % ((((lo[2] + hi[2]) / 2) - lo[2]) / sz))
    print('hole diameter as frac of width = %.6f' % (85.22 / sx))


emit_fractions()
