"""
Convierte los VRML exportados desde NX (modelos originales UR5 y Turin TCR050) en el JSON que usa la simulación.

- Cada cuerpo (Body__xxxxx) se asigna a un eslabón del robot según su posición en la pose de exportación.
- Las coordenadas se llevan a un marco Z-arriba con la base del robot en el origen (mm).
- Los ejes de articulación se expresan en ese mismo marco, en la pose de exportación (q = 0).
- Para cada eslabón se calculan esferas envolventes para la verificación de colisiones.

Uso:  python tools/build_models.py <UR3D.wrl> <TURIN3DVRML>
Salida: src/models/robots.json
"""
import sys, json, base64, math, os
from array import array
from vrml_parse import parse

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'src', 'models', 'robots.json')

# ---------------------------------------------------------------- UR5 (CB) -----------------------------------
# Pose exportada: brazo vertical (la "posición de transporte" del UR). Base en el origen, Z arriba.
# Ejes según la tabla DH oficial del UR5: d1=89.159, a2=425, a3=392.25, d4=109.15, d5=94.65, d6=82.3,
# verificados contra los centros de las tapas en la malla (diferencias < 1 mm).
UR = {
    'name': 'UR5',
    'transform': lambda p: p,  # ya está en Z arriba con la base en el origen
    'joints': [
        {'point': [0, 0, 0],            'axis': [0, 0, 1]},
        {'point': [0, 0, 89.159],       'axis': [0, -1, 0]},
        {'point': [0, 0, 514.159],      'axis': [0, -1, 0]},
        {'point': [0, 0, 906.409],      'axis': [0, -1, 0]},
        {'point': [0, -109.15, 0],      'axis': [0, 0, -1]},
        {'point': [0, 0, 1001.059],     'axis': [0, -1, 0]},
    ],
    # brida: centro de la cara de montaje y dirección de la herramienta (eje Z de herramienta)
    'flange': {'point': [0, -190.9, 1001.059], 'z': [0, -1, 0], 'x': [-1, 0, 0]},
    'limits_deg': [[-360, 360]] * 6,
    'links': {
        0: ['Body__53423'],
        1: ['Body__53425', 'Body__53426', 'Body__53427', 'Body__53428'],
        2: ['Body__53429', 'Body__53431', 'Body__53430'],
        3: ['Body__53432', 'Body__53433', 'Body__53434', 'Body__53435'],
        4: ['Body__53436', 'Body__53437'],
        5: ['Body__53439', 'Body__53438'],
        6: ['Body__53440'],
    },
    # colores tomados de la captura de NX (carcasas negras, tapas celestes, tubos gris claro, brida plateada)
    'colors': {
        'Body__53423': 'base', 'Body__53425': 'housing', 'Body__53426': 'cap', 'Body__53427': 'housing',
        'Body__53428': 'cap', 'Body__53429': 'tube', 'Body__53431': 'housing', 'Body__53430': 'cap',
        'Body__53432': 'housing', 'Body__53433': 'tube', 'Body__53434': 'housing', 'Body__53435': 'cap',
        'Body__53436': 'housing', 'Body__53437': 'cap', 'Body__53439': 'housing', 'Body__53438': 'cap',
        'Body__53440': 'flange',
    },
    'palette': {'base': '#c9ccd0', 'housing': '#2b2e33', 'cap': '#3fc9d4', 'tube': '#b9bec4', 'flange': '#d4d7da'},
}

# ---------------------------------------------------------------- Turin TCR050 -------------------------------
# En la escena de NX el Turin está montado invertido (base hacia +Z, brazo hacia -Z) con la base en (0, 102.7, 0).
# Se gira 180° sobre X y se traslada para dejarlo de pie con la base en el origen:  (x, y, z) -> (x, 102.7 - y, -z).
# Los ejes surgen de los centros de las tapas/anillos de cada articulación medidos en la malla.
TY = 102.7
TURIN = {
    'name': 'TCR050',
    'transform': lambda p: (p[0], TY - p[1], -p[2]),
    'joints': [
        {'point': [0, 0, 0],                   'axis': [0, 0, 1]},
        {'point': [69, 0, 107.0],              'axis': [1, 0, 0]},
        {'point': [69, 0, 546.5],              'axis': [1, 0, 0]},
        {'point': [59.5, TY - 455.7, 546.5],   'axis': [1, 0, 0]},
        {'point': [110.5, 0, 546.5],           'axis': [0, 1, 0]},
        {'point': [110.5, TY - 563.25, 0],     'axis': [0, 0, -1]},
    ],
    'flange': {'point': [110.5, TY - 563.25, 454.0], 'z': [0, 0, -1], 'x': [1, 0, 0]},
    'limits_deg': [[-360, 360], [-360, 360], [-165, 165], [-360, 360], [-360, 360], [-360, 360]],
    'links': {
        0: ['Body__53517'],
        1: ['Body__53516', 'Body__54150', 'Body__54149'],
        2: ['Body__63308', 'Body__66824', 'Body__62111', 'Body__62959', 'Body__65162'],
        3: ['Body__68471', 'Body__66825', 'Body__68756', 'Body__68757', 'Body__74023'],
        4: ['Body__71788', 'Body__74116'],
        5: ['Body__74117', 'Body__77212'],
        6: ['Body__80257', 'Body__77661', 'Body__80256', 'Body__77213'],
    },
    # colores según la captura: placa base, brazo, antebrazo y muñeca en amarillo; articulaciones gris metalizado
    'colors': {
        'Body__53517': 'yellow', 'Body__53516': 'ring', 'Body__54150': 'gray', 'Body__54149': 'ring',
        'Body__63308': 'gray', 'Body__66824': 'yellowDark', 'Body__62111': 'yellow', 'Body__62959': 'yellowDark',
        'Body__65162': 'gray', 'Body__68471': 'ring', 'Body__66825': 'yellow', 'Body__68756': 'ring',
        'Body__68757': 'gray', 'Body__74023': 'ring', 'Body__71788': 'gray', 'Body__74116': 'ring',
        'Body__74117': 'yellow', 'Body__77212': 'ring', 'Body__80257': 'gray', 'Body__77661': 'dark',
        'Body__80256': 'dark', 'Body__77213': 'dark',
    },
    'palette': {'yellow': '#e9c41c', 'yellowDark': '#c9a514', 'gray': '#a7b2b6', 'ring': '#7f8a8e', 'dark': '#3a3f44'},
}


def b64(arr):
    return base64.b64encode(arr.tobytes()).decode('ascii')


def spheres_for(points, cell=55.0):
    """Esferas envolventes por celdas: cada celda de 'cell' mm con vértices genera una esfera que los contiene."""
    cells = {}
    for p in points:
        k = (math.floor(p[0] / cell), math.floor(p[1] / cell), math.floor(p[2] / cell))
        cells.setdefault(k, []).append(p)
    out = []
    for pts in cells.values():
        c = [sum(q[i] for q in pts) / len(pts) for i in range(3)]
        r = max(math.dist(c, q) for q in pts)
        out.append([round(c[0], 1), round(c[1], 1), round(c[2], 1), round(max(r, 8.0) + 2.0, 1)])
    return out


def build(robot, path):
    bodies = {b['name']: b for b in parse(path)}
    assigned = {n for names in robot['links'].values() for n in names}
    missing = set(bodies) - assigned
    if missing:
        raise SystemExit(f"{robot['name']}: cuerpos sin asignar {sorted(missing)}")
    links = []
    for li in range(7):
        parts = []
        all_pts = []
        for name in robot['links'][li]:
            b = bodies[name]
            P = b['points']
            pos = array('f')
            # soldar vértices duplicados (NX repite los vértices por cara)
            remap, uniq = [], {}
            for i in range(0, len(P), 3):
                q = robot['transform']((P[i], P[i + 1], P[i + 2]))
                key = (round(q[0], 3), round(q[1], 3), round(q[2], 3))
                j = uniq.get(key)
                if j is None:
                    j = len(uniq); uniq[key] = j; pos.extend(key); all_pts.append(key)
                remap.append(j)
            tris = [remap[t] for t in b['tris']]
            # el giro de 180° no invierte la orientación (det = +1), así que el sentido de las caras se conserva
            idx = array('I', tris) if len(uniq) > 65535 else array('H', tris)
            parts.append({'name': name, 'color': robot['palette'][robot['colors'][name]],
                          'positions': b64(pos), 'indices': b64(idx), 'index32': idx.typecode == 'I'})
        links.append({'parts': parts, 'spheres': spheres_for(all_pts)})
    return {
        'name': robot['name'],
        'joints': robot['joints'],
        'flange': robot['flange'],
        'limits': [[math.radians(a), math.radians(b)] for a, b in robot['limits_deg']],
        'links': links,
    }


if __name__ == '__main__':
    ur_path, turin_path = sys.argv[1], sys.argv[2]
    data = {'UR5': build(UR, ur_path), 'TCR050': build(TURIN, turin_path)}
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w') as f:
        json.dump(data, f, separators=(',', ':'))
    for k, r in data.items():
        print(k, 'eslabones:', len(r['links']), 'esferas:', [len(l['spheres']) for l in r['links']])
    print('->', os.path.normpath(OUT), os.path.getsize(OUT) // 1024, 'KB')
