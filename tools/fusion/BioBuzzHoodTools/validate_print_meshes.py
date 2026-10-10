"""Check binary STL topology. Units and mechanical fit need independent verification.

Pass --recursive to include nested exports, or --manifest JSON_LIST to require
a nonempty set of relative STL paths. Validation also runs under python -O.
"""
from pathlib import Path
import struct
import collections
import json
import sys
import argparse
import math

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('folder', type=Path)
parser.add_argument('--recursive', action='store_true')
parser.add_argument('--manifest', type=Path)
args = parser.parse_args()
folder = args.folder.resolve()
if not folder.is_dir():
    parser.error('Input folder must exist')
report_path = folder/'mesh_validation.json'
# Replace stale success evidence before validation begins.
report_path.write_text(json.dumps({'status':'FAILED', 'error':'Validation incomplete'}), encoding='utf-8')
if args.manifest:
    names = json.loads(args.manifest.read_text(encoding='utf-8'))
    if not isinstance(names, list) or not names or not all(isinstance(n,str) for n in names):
        parser.error('Manifest must be a nonempty JSON list of STL paths')
    paths = [(folder/n).resolve() for n in names]
    if len(set(paths)) != len(paths) or any(not p.is_relative_to(folder) or p.suffix.lower() != '.stl' for p in paths):
        parser.error('Manifest must contain unique STL paths inside the input folder')
else:
    paths = sorted(p for p in (folder.rglob('*') if args.recursive else folder.iterdir())
                   if p.is_file() and p.suffix.lower() == '.stl')
if not paths:
    parser.error('No STL files selected; use --recursive for nested exports')
results = []
for path in paths:
    blob = path.read_bytes()
    if len(blob) < 84:
        raise ValueError(f'{path.name}: truncated binary STL')
    count, = struct.unpack_from('<I',blob,80)
    if count == 0 or len(blob) != 84+count*50:
        raise ValueError(f'{path.name}: empty or invalid binary STL length')
    edges = collections.Counter()
    orientations = collections.Counter()
    neighbours = collections.defaultdict(set)
    points = set()
    volume = 0
    for index in range(count):
        vals = struct.unpack_from('<12fH',blob,84+index*50)
        if not all(math.isfinite(v) for v in vals[:12]):
            raise ValueError(f'{path.name}: nonfinite triangle')
        vs = [tuple(round(x,5) for x in vals[j:j+3]) for j in (3,6,9)]
        if len(set(vs)) != 3:
            raise ValueError(f'{path.name}: degenerate triangle')
        points.update(vs)
        a,bb,c = vs
        ab = [bb[i]-a[i] for i in range(3)]
        ac = [c[i]-a[i] for i in range(3)]
        cross = [ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]]
        if not any(v != 0 for v in cross):
            raise ValueError(f'{path.name}: zero-area triangle')
        volume += (a[0]*(bb[1]*c[2]-bb[2]*c[1])+a[1]*(bb[2]*c[0]-bb[0]*c[2])+a[2]*(bb[0]*c[1]-bb[1]*c[0]))/6
        for a,bb in zip(vs,vs[1:]+vs[:1]):
            key = tuple(sorted((a,bb)))
            edges[key] += 1
            orientations[key] += 1 if a<bb else -1
            neighbours[a].add(bb)
            neighbours[bb].add(a)
    remaining = set(points)
    shells = 0
    while remaining:
        shells += 1
        todo = [remaining.pop()]
        while todo:
            for vertex in neighbours[todo.pop()]:
                if vertex in remaining:
                    remaining.remove(vertex)
                    todo.append(vertex)
    result = {'file':path.relative_to(folder).as_posix(),'triangles':count,'closed_two_faces_per_edge':all(v==2 for v in edges.values()),
              'consistent_winding':all(v==0 for v in orientations.values()),'connected_shells':shells,
              'volume_mm3':volume,'bounds_mm':{'min':[min(p[i] for p in points) for i in range(3)],
                                             'max':[max(p[i] for p in points) for i in range(3)]}}
    if not (result['closed_two_faces_per_edge'] and result['consistent_winding'] and shells==1
            and math.isfinite(volume) and volume>0):
        raise ValueError(f'{path.name}: invalid topology: {result}')
    results.append(result)
report_path.write_text(json.dumps(results,indent=2), encoding='utf-8')
print(json.dumps(results,indent=2))
