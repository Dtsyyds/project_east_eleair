"""No hardware: replace canonical destinations, verify exact argv/exit/signal."""
import json,os,subprocess,sys,time
from pathlib import Path
import pytest
SYSTEM=Path(__file__).resolve().parents[1]
DEST={'collect':'start_vehicle_collection.sh','stop':'stop_vehicle_collection.sh',
      'replay':'replay_vehicle_collection.sh','validate':'validate_vehicle_collection.py',
      'handoff':'build_role_handoff.py'}
@pytest.mark.parametrize('name',list(DEST))
def test_args_exit_and_safe_help(tmp_path,name):
    scripts=tmp_path/'scripts';scripts.mkdir()
    report=tmp_path/'argv.json'
    body="import json,sys\nfrom pathlib import Path\nPath("+repr(str(report))+").write_text(json.dumps(sys.argv[1:]))\nraise SystemExit(17)\n"
    dest=scripts/DEST[name]
    if dest.suffix=='.py':dest.write_text(body)
    else:
        impl=tmp_path/'impl.py';impl.write_text(body)
        dest.write_text('#!/bin/bash\nexec '+sys.executable+' '+str(impl)+' "$@"\n')
    env=dict(os.environ,FIELD_PHYSIC_ROOT=str(tmp_path))
    args=['a b','--name','literal$(never_run)', 'x:=y']
    result=subprocess.run(['bash',str(SYSTEM/'scripts'/f'{name}.sh'),*args],env=env,capture_output=True,text=True)
    assert result.returncode==17,result.stderr
    assert json.loads(report.read_text())==args
    result=subprocess.run(['bash',str(SYSTEM/'scripts'/f'{name}.sh'),'--help'],env=env,capture_output=True,text=True)
    assert result.returncode==0 and 'canonical entry' in result.stdout

def test_term_reaches_exec_destination(tmp_path):
    scripts=tmp_path/'scripts';scripts.mkdir()
    ready=tmp_path/'ready';stopped=tmp_path/'stopped'
    impl=tmp_path/'wait.py'
    impl.write_text("import signal,time\nfrom pathlib import Path\ndef stop(*args):\n Path("+repr(str(stopped))+").write_text('TERM')\n raise SystemExit(23)\nsignal.signal(signal.SIGTERM,stop)\nPath("+repr(str(ready))+").touch()\ntime.sleep(10)\n")
    (scripts/DEST['stop']).write_text('#!/bin/bash\nexec '+sys.executable+' '+str(impl)+' "$@"\n')
    p=subprocess.Popen(['bash',str(SYSTEM/'scripts/stop.sh'),'fixture'],env=dict(os.environ,FIELD_PHYSIC_ROOT=str(tmp_path)),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        deadline=time.monotonic()+5
        while not ready.exists() and time.monotonic()<deadline:time.sleep(.02)
        assert ready.exists()
        p.terminate();p.communicate(timeout=2)
        assert p.returncode==23 and stopped.read_text()=='TERM'
    finally:
        if p.poll() is None:p.kill();p.wait()
