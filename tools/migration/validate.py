#!/usr/bin/env python3
"""Non-device migration integrity and syntax checks; never imports project/device code."""
import argparse
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
BASELINE = 'b34a3c29c62500915c302b340cb4d51880b00980'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    ledger = json.loads((ROOT/'docs/architecture/migration_files.json').read_text())['files']
    errors, excluded, blocked = [], [], []
    sources = subprocess.check_output(['git','ls-tree','-rz','--name-only',BASELINE],cwd=ROOT).decode().split('\0')[:-1]
    if set(sources) != {x['source'] for x in ledger}: errors.append('baseline source set differs from ledger')
    targets = [x['target'] for x in ledger]
    if len(set(targets)) != len(targets): errors.append('duplicate canonical ledger target')
    counts = Counter()
    for row in ledger:
        path = ROOT/row['target']
        if not path.is_file(): errors.append('missing: '+row['target']); continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        counts['unchanged_content' if digest == row['source_sha256'] else 'changed_content'] += 1
        if Path(row['source']).suffix in ('.msg','.srv','.action') and digest != row['source_sha256']:
            errors.append('interface changed: '+row['source'])
        for extra in row.get('split_targets',[]):
            if not (ROOT/extra).is_file():errors.append('missing extracted module: '+extra)
    supplements = json.loads((ROOT/'docs/architecture/migration_supplemental_files.json').read_text())
    for row in supplements:
        path=ROOT/row['target']
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=row.get('target_sha256',row['sha256']):
            errors.append('supplemental source mismatch: '+row['target'])
    packages = {}
    for path in sorted((ROOT/'ros2_ws/src').rglob('package.xml')):
        root = ET.parse(path).getroot(); name=root.findtext('name')
        if name in packages: errors.append('duplicate ROS package: '+name)
        packages[name]={'path': str(path.parent.relative_to(ROOT)), 'dependencies': [{'tag':x.tag,'name':x.text} for x in root if x.tag.endswith('depend')]}
    original = json.loads((ROOT/'docs/architecture/ros_packages.json').read_text())['original_packages']
    for item in original:
        before = ET.fromstring(subprocess.check_output(['git','show',BASELINE+':'+item['source']],cwd=ROOT))
        after = ET.parse(ROOT/item['target']).getroot()
        if before.findtext('name') != after.findtext('name'): errors.append('ROS name changed: '+item['source'])
        olddeps={(x.tag,x.text) for x in before if x.tag.endswith('depend')}
        newdeps={(x.tag,x.text) for x in after if x.tag.endswith('depend')}
        if olddeps-newdeps: errors.append('removed ROS dependency: '+item['target']+' '+str(olddeps-newdeps))
    # All project sources, including migrated upstream ROS Python; third_party is preserved upstream.
    import yaml
    import toml
    skip_parts={'.git','.runtime','__pycache__','.pytest_cache','node_modules','.egg-info'}
    for path in sorted(ROOT.rglob('*')):
        rel=path.relative_to(ROOT)
        if not path.is_file() or any(p in skip_parts or p.endswith('.egg-info') for p in rel.parts):continue
        if rel.parts[0]=='third_party' or str(rel).startswith('docs/history/'):
            continue
        suffix=path.suffix
        try:
            if suffix=='.py': ast.parse(path.read_text(),filename=str(rel));counts['python']+=1
            elif suffix=='.sh':
                result=subprocess.run(['bash','-n',str(path)],capture_output=True,text=True)
                if result.returncode:raise ValueError(result.stderr)
                counts['bash']+=1
            elif suffix=='.js' and not shutil.which('node'):
                blocked.append(str(rel)+': Node.js unavailable; syntax/test not executed')
            elif suffix=='.js' and shutil.which('node'):
                result=subprocess.run(['node','--check',str(path)],capture_output=True,text=True)
                if result.returncode:raise ValueError(result.stderr)
                counts['javascript']+=1
            elif suffix=='.json': json.loads(path.read_text());counts['json']+=1
            elif suffix in ('.yaml','.yml'):
                # OpenCV FileStorage uses a non-standard YAML 1.0 directive.
                text=path.read_text().replace('%YAML:1.0','')
                if 'opencv-matrix' in text:
                    loader=type('OpenCVLoader',(yaml.SafeLoader,),{})
                    loader.add_constructor('tag:yaml.org,2002:opencv-matrix',lambda loader,node:loader.construct_mapping(node))
                    list(yaml.load_all(text,Loader=loader))
                else:list(yaml.safe_load_all(text))
                counts['yaml']+=1
            elif suffix=='.toml':toml.loads(path.read_text());counts['toml']+=1
            elif path.name=='package.xml':ET.parse(path);counts['package_xml']+=1
        except Exception as exc:errors.append(str(rel)+': '+str(exc))
    excluded=['third_party: upstream examples/source unchanged; syntax of all vendor examples is outside project Python support','docs/history: immutable historical snapshots and archived obsolete tools; not executable entrypoints','C/C++: no compilation performed; syntax/build deferred','ROS install resources and generated messages require the next clean build']
    report={'baseline':BASELINE,'baseline_files':len(sources),'original_ros_packages':len(original),'discovered_ros_packages':len(packages),'counts':dict(counts),'packages':packages,'errors':errors,'excluded_or_deferred':excluded,'dependency_blocked_checks':blocked,'supplemental_files':len(supplements),'status':('passed_available_checks' if blocked else 'passed') if not errors else 'failed','scope':'static only; no hardware or full build'}
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='packages'},ensure_ascii=False,indent=2))
    return bool(errors)
if __name__=='__main__':raise SystemExit(main())
