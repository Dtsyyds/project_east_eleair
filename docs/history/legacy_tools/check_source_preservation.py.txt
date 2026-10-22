"""Read-only comparison of the frozen historical stat inventory; no media writes."""
import argparse,json,os
from pathlib import Path
SYSTEM=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path)
p.add_argument('--update-closeout',action='store_true')
args=p.parse_args()
if not args.output and not args.update_closeout:p.error('--output or --update-closeout required')
root=Path(os.getenv('FIELD_DATA_ROOT','/media/mcap_data'))/'runs'
if args.output and args.output.resolve().is_relative_to(root.resolve()):p.error('report must be outside historical runs')
if args.output and args.output.exists():p.error('refusing to overwrite an existing report')
before=json.loads((SYSTEM/'reports/refactor_v1/historical_stat_before.json').read_text())
frozen_root=Path('/media/mcap_data/runs')
if root!=frozen_root:
    before={str(root/Path(name).relative_to(frozen_root)):value for name,value in before.items()}
available=root.is_dir()
after={str(f):[f.stat().st_size,f.stat().st_mtime_ns] for f in root.rglob('*') if f.is_file()} if available else {}
changed=[name for name,value in before.items() if after.get(name)!=value] if available else []
result={'status':'verified_stat_metadata' if available else 'environment_blocked',
        'media_directory_available':available,'comparison_root':str(root),'inventory_root':str(frozen_root),'source_data_unchanged':available and before==after,
        'initial_files':len(before),'current_files':len(after) if available else None,
        'changed_or_missing':changed,'added':sorted(set(after)-set(before)),
        'note':'mount unavailable does not mean files were deleted; size/mtime comparison is not a full-byte SHA audit'}
if args.output:
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
if args.update_closeout:
    report=SYSTEM/'reports/refactor_v1/final_report.json'
    d=json.loads(report.read_text());d.setdefault('closeout',{}).setdefault('checks',{})['source_preservation']={**result,'status':'verified' if result['source_data_unchanged'] else 'environment_blocked' if not available else 'failed','byte_sha_audit_verified':False}
    report.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['source_data_unchanged'] else 2)
