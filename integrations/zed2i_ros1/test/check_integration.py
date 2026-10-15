#!/usr/bin/env python3
"""Portable-copy checks. No real ROS processes, camera access or system changes."""
import ast
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
for path in ROOT.rglob('*.py'):
    ast.parse(path.read_text(), filename=str(path))
for path in ROOT.rglob('*.sh'):
    subprocess.run(['bash', '-n', str(path)], check=True)
for pattern in ('*.launch', 'package.xml'):
    for path in ROOT.rglob(pattern):
        ET.parse(path)
config = json.loads((ROOT/'zed_time_bridge/config/shadow.json').read_text())
assert config['compensation_ns'] == 0 and config['expected_clock_source'] == 2
launch = ET.parse(ROOT/'l1/zed_l1.launch').getroot()
assert launch.find("arg[@name='serial_number']").get('default') == '0'
assert launch.find("param[@name='/zed2i/zed_node/general/serial_number']").get('value') == '$(arg serial_number)'

# Mock only the external commands; exercise paths with spaces and a foreign cwd.
with tempfile.TemporaryDirectory(prefix='zed portable ') as folder:
    temp = Path(folder)
    ws = temp/'isolated ws'
    (ws/'devel').mkdir(parents=True)
    (ws/'devel/setup.bash').write_text('export MOCK_SETUP_LOADED=yes\n')
    binary = temp/'bin'; binary.mkdir()
    capture = temp/'arguments.json'
    for name in ('roslaunch', 'rosbag'):
        tool = binary/name
        tool.write_text('#!/usr/bin/python3\nimport json,os,sys\n'
                        'from pathlib import Path\n'
                        'Path(os.environ["MOCK_CAPTURE"]).write_text(json.dumps(dict('
                        'argv=sys.argv[1:], master=os.environ["ROS_MASTER_URI"], '
                        'setup=os.environ["MOCK_SETUP_LOADED"])))\n')
        tool.chmod(0o755)
    env = dict(os.environ, ZED_WS=str(ws), ZED_EVIDENCE_DIR=str(temp/'evidence'),
               ROS_MASTER_URI='http://localhost:11439', MOCK_CAPTURE=str(capture),
               PATH=str(binary)+os.pathsep+os.environ['PATH'])
    def run(script, *args, good=True):
        result = subprocess.run(['bash', str(ROOT/'l1'/script), *args], env=env,
                                cwd=temp, capture_output=True, text=True)
        assert (result.returncode == 0) == good, result.stderr
        if good:
            data = json.loads(capture.read_text())
            assert data['setup'] == 'yes' and data['master'] == env['ROS_MASTER_URI']
            return data['argv']
    assert run('run_camera.sh', 'serial_number:=123') == [str(ROOT/'l1/zed_l1.launch'), 'serial_number:=123']
    args = run('record_bag.sh', str(temp/'new bag.bag'))
    assert args[:4] == ['record', '--lz4', '--duration=5', '--buffsize=2048']
    assert args[args.index('-O')+1] == str(temp/'new bag.bag')
    run('record_bag.sh', 'relative.bag', good=False)
    existing = temp/'existing.bag'; existing.write_bytes(b'preserved')
    run('record_bag.sh', str(existing), good=False)
    assert existing.read_bytes() == b'preserved'
    # --anchor must never abbreviate the separate --anchor-module-root option.
    # No ROS subscriptions: use a missing anchor in this temporary directory only.
    protocol = ROOT.parents[1]/'src/sensor_time_bridge/src'
    env['SENSOR_TIME_BRIDGE_PYTHON_ROOT'] = str(protocol)
    env.pop('PYTHONPATH', None)
    script = ROOT/'zed_time_bridge/scripts/zed_time_audit.py'
    for name, extra, imported in [('env', [], False),
                                  ('explicit', ['--anchor-module-root', str(protocol)], False),
                                  ('installed', [], True)]:
        local = dict(env)
        if name != 'env':
            local.pop('SENSOR_TIME_BRIDGE_PYTHON_ROOT')
        if imported:
            local['PYTHONPATH'] = str(protocol)
        output = temp/name
        result = subprocess.run(['/usr/bin/python3', '-B', str(script), 'record',
                                 '--duration', '0.05', '--anchor', str(temp/'missing'),
                                 '--outdir', str(output), *extra], env=local, cwd=temp,
                                capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        summary = json.loads(result.stdout)
        assert summary['events'] == 0
        replay = subprocess.run(['/usr/bin/python3', '-B', str(script), 'replay',
                                 '--input-dir', str(output), '--outdir', str(temp/(name+'_replay'))],
                                env=env, cwd=temp, capture_output=True, text=True)
        assert replay.returncode == 0, replay.stderr
print('PASS: syntax/config, portable launch/bag guards, three protocol import modes, --anchor parsing and replay')
