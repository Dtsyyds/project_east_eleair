"""No hardware: replace canonical destinations, verify exact argv/exit/signal."""
from inspection_runtime.paths import project_root
import json,os,subprocess,sys,time
import shlex
from pathlib import Path
import pytest
SYSTEM=project_root() / 'deploy'
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
    env=dict(os.environ)
    invocation=[sys.executable, '-c', "from inspection_runtime.commands import COMMANDS, dispatch; import sys; COMMANDS["+repr(name)+"] = ('script', "+repr(str(dest))+ "); dispatch("+repr(name)+", sys.argv[1:])"]
    if dest.suffix=='.py':
        wrapper=scripts/(name+'.sh'); wrapper.write_text('#!/bin/bash\nexec '+shlex.quote(sys.executable)+' '+shlex.quote(str(dest))+' "$@"\n')
        invocation=[sys.executable, '-c', "from inspection_runtime.commands import COMMANDS, dispatch; import sys; COMMANDS["+repr(name)+"] = ('script', "+repr(str(wrapper))+ "); dispatch("+repr(name)+", sys.argv[1:])"]
    args=['a b','--name','literal$(never_run)', 'x:=y']
    result=subprocess.run(invocation+args,env=env,capture_output=True,text=True)
    assert result.returncode==17,result.stderr
    assert json.loads(report.read_text())==args
    result=subprocess.run(['bash',str(project_root() / 'deploy/compat/system'/f'{name}.sh'),'--help'],env=env,capture_output=True,text=True)
    assert result.returncode==0 and 'usage' in result.stdout.lower()

def test_term_reaches_exec_destination(tmp_path):
    scripts=tmp_path/'scripts';scripts.mkdir()
    ready=tmp_path/'ready';stopped=tmp_path/'stopped'
    impl=tmp_path/'wait.py'
    impl.write_text("import signal,time\nfrom pathlib import Path\ndef stop(*args):\n Path("+repr(str(stopped))+").write_text('TERM')\n raise SystemExit(23)\nsignal.signal(signal.SIGTERM,stop)\nPath("+repr(str(ready))+").touch()\ntime.sleep(10)\n")
    (scripts/DEST['stop']).write_text('#!/bin/bash\nexec '+sys.executable+' '+str(impl)+' "$@"\n')
    invocation=[sys.executable,'-c',"from inspection_runtime.commands import COMMANDS,dispatch; COMMANDS['stop']=('script',"+repr(str(scripts/DEST['stop']))+"); dispatch('stop',['fixture'])"]
    p=subprocess.Popen(invocation,env=dict(os.environ),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        deadline=time.monotonic()+5
        while not ready.exists() and time.monotonic()<deadline:time.sleep(.02)
        assert ready.exists()
        p.terminate();p.communicate(timeout=2)
        assert p.returncode==23 and stopped.read_text()=='TERM'
    finally:
        if p.poll() is None:p.kill();p.wait()
