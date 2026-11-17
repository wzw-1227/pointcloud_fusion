#!/usr/bin/env python3
"""Apply the recorded LIO patch only to its exact baseline; preserve unknown edits."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('/home/yang/wzw/ct_ws/src/LIO-SAM'))
    p.add_argument('--patches',type=Path);a=p.parse_args()
    if a.patches is None:
        candidate=Path(__file__).resolve().parents[1]/'patches'
        if candidate.is_dir():a.patches=candidate
        else:
            import rospkg
            a.patches=Path(rospkg.RosPack().get_path('start_up'))/'patches'
    record=json.loads((a.patches/'lio_source_record.json').read_text());target=a.source/'src/mapOptmization.cpp'
    if digest(target)==record['patched_map_sha256']:
        if digest(a.source/'include/scanStateGuard.h')!=record['helper_sha256']:p.error('patched source helper hash mismatch')
        print('Recorded patch already applied');return
    if digest(target)!=record['original_map_sha256']:p.error('unknown source changes; refusing to overwrite. Review lio_scan_state.patch against your source first')
    if digest(a.patches/'lio_scan_state.patch')!=record['patch_sha256']:p.error('patch hash mismatch')
    if digest(a.patches/'scanStateGuard.h')!=record['helper_sha256']:p.error('helper hash mismatch')
    subprocess.run(['patch','--dry-run','--batch','-p1','-i',str((a.patches/'lio_scan_state.patch').resolve())],cwd=a.source,check=True)
    shutil.copyfile(a.patches/'scanStateGuard.h',a.source/'include/scanStateGuard.h')
    subprocess.run(['patch','--batch','-p1','-i',str((a.patches/'lio_scan_state.patch').resolve())],cwd=a.source,check=True)
    if digest(target)!=record['patched_map_sha256']:raise RuntimeError('post-patch source hash mismatch')
    print('Applied; rebuild lio_sam in the LIO workspace before launching')
if __name__=='__main__':main()
