#!/usr/bin/env python3
"""Patch a build copy; the pinned source archive remains untouched."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

p = argparse.ArgumentParser()
p.add_argument('source', type=Path)
p.add_argument('destination', type=Path)
p.add_argument('patch', type=Path)
a = p.parse_args()
record = json.loads((a.source.parent/'EHPR_SOURCE.json').read_text())
assert hashlib.sha256((a.source/'include/VoxelMap.hpp').read_bytes()).hexdigest() == record['voxel_header_sha256']
core_record = json.loads((a.source.parent/'EHPR_STAGE3_SOURCE.json').read_text())
assert core_record['upstream_commit'] == record['commit']
for name, digest in core_record['files_sha256'].items():
    if hashlib.sha256((a.source/name).read_bytes()).hexdigest() != digest:
        raise RuntimeError('Pinned upstream source was changed: '+name)
if a.destination.exists():
    shutil.rmtree(a.destination)
shutil.copytree(a.source, a.destination)
subprocess.run(['patch', '--batch', '-p1', '-i', str(a.patch.resolve())], cwd=a.destination, check=True)
(a.destination/'STAGE3_PATCH.json').write_text(json.dumps({
    'upstream_commit': record['commit'], 'patch_sha256': hashlib.sha256(a.patch.read_bytes()).hexdigest(),
    'original_core_sha256': core_record['files_sha256'],
    'compiled_core_sha256': {name: hashlib.sha256((a.destination/name).read_bytes()).hexdigest()
                             for name in core_record['files_sha256']}
}, indent=2)+'\n')
