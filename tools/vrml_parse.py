"""Lector mínimo de VRML 2.0 exportado por NX: Transform DEF + Shape(Material, IndexedFaceSet)."""
import re, sys, json

def parse(path):
    txt = open(path, encoding='utf-8', errors='replace').read()
    bodies = []
    # cada cuerpo empieza con "DEF <name> Transform"
    parts = re.split(r'DEF\s+(\S+)\s+Transform', txt)
    for i in range(1, len(parts), 2):
        name, body = parts[i], parts[i+1]
        col = re.search(r'diffuseColor\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)', body)
        tr = re.search(r'translation\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)', body)
        rot = re.search(r'rotation\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)', body)
        sc = re.search(r'scale\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)', body)
        P = []; tris = []
        # un cuerpo NX contiene un Shape por cara: se juntan todos en una sola malla
        for pts, idx in re.findall(r'point\s*\[([^\]]*)\].*?coordIndex\s*\[([^\]]*)\]', body, re.S):
            base = len(P) // 3
            P += [float(v) for v in re.findall(r'[-+]?[\d.]+(?:[eE][-+]?\d+)?', pts)]
            face = []
            for v in (int(t) for t in re.findall(r'-?\d+', idx)):
                if v < 0:
                    for k in range(1, len(face) - 1): tris += [base + face[0], base + face[k], base + face[k+1]]
                    face = []
                else: face.append(v)
            for k in range(1, len(face) - 1): tris += [base + face[0], base + face[k], base + face[k+1]]
        bodies.append(dict(name=name, color=[float(c) for c in col.groups()] if col else None,
                           translation=[float(c) for c in tr.groups()] if tr else None,
                           rotation=[float(c) for c in rot.groups()] if rot else None,
                           scale=[float(c) for c in sc.groups()] if sc else None,
                           points=P, tris=tris))
    return bodies

if __name__ == '__main__':
    for b in parse(sys.argv[1]):
        P = b['points']; xs, ys, zs = P[0::3], P[1::3], P[2::3]
        print(f"{b['name']:14s} col={b['color']} T={b['translation']} R={b['rotation']} S={b['scale']} nv={len(xs)} nt={len(b['tris'])//3} "
              f"x[{min(xs):8.1f},{max(xs):8.1f}] y[{min(ys):8.1f},{max(ys):8.1f}] z[{min(zs):8.1f},{max(zs):8.1f}]")
