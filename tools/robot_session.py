"""RDK 单入口 supervisor：现有相机系统 + 底盘/循迹/分模块录制。

不自动 enable，不执行任意网页命令，不终止其他终端已有的 launch。
--plan 不要求 ROS/设备在线；--check 只预检，不打开串口或启动电机。
"""

import argparse
import contextlib
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--chassis-workspace', type=Path,
                        default=Path(os.environ.get('CHASSIS_WS',
                        '/home/sunrise/Desktop/Robot_Chassis_Code/robot_chassis_ros2ctrl')))
    parser.add_argument('--platform', choices=('s100p', 'rk3588', 'x86'),
                        default=os.environ.get('INSPECTION_PLATFORM', 's100p'))
    parser.add_argument('--camera-mode', choices=('all', 'd405', 'cameras'), default='all',
                        help='默认 all 启动三相机、MID360/IMU 和涡流；d405/cameras 用于单设备或三相机调试')
    parser.add_argument('--web-root', type=Path, default=ROOT / 'apps/web_console',
                        help='网页目录，可选择独立的 web_console_chassis_tracking 扩展版')
    parser.add_argument('--no-camera', action='store_true', help='相机由其他终端启动；本入口只提供新网页和底盘')
    parser.add_argument('--drive-mode', choices=('tracked_2wd', 'four_wheel'), default='tracked_2wd')
    parser.add_argument('--real', action='store_true', help='允许实际循迹输出；仍须在网页人工启动')
    parser.add_argument('--domain', type=int, default=77)
    parser.add_argument('--feedback-timeout-ms', type=int, default=500)
    parser.add_argument('--port', type=int, default=9001, help='沿用原上位机 WebSocket 通信端口')
    parser.add_argument('--http-port', type=int, default=8080, help='沿用原上位机网页端口')
    parser.add_argument('--camera-http-port', type=int, default=8081, help='相机后台 HTTP 内部转发端口')
    parser.add_argument('--camera-port', type=int, default=9003, help='仅供网关转发的相机后台端口')
    parser.add_argument('--robot-port', type=int, default=9002)
    parser.add_argument('--host', action='append', help='可重复指定访问地址；省略时同时支持网线 192.168.127.10 和无线 192.168.1.148')
    parser.add_argument('--tracking-params', type=Path, help='使用现场现有 PID/置信度/超时配置，不自动覆盖')
    parser.add_argument('--data-root', type=Path, default=Path(os.environ.get('FIELD_DATA_ROOT', '/media/mcap_data')))
    parser.add_argument('--mcap-modules', default='all')
    parser.add_argument('--no-record', action='store_true')
    parser.add_argument('--plan', action='store_true')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args(argv)
    # 绑定仍为所有网卡，但 WebSocket Origin 必须同时允许两种网页访问地址。
    # 显式 --host 保留现场自定义能力，可重复添加多个；不使用通配符放开任意网站。
    args.hosts = list(dict.fromkeys(args.host or ['192.168.127.10', '192.168.1.148']))
    args.host = args.hosts[0]
    if not 0 <= args.domain <= 232:
        parser.error('ROS_DOMAIN_ID 应为 0..232；须与相机/电脑一致')
    if not 1 <= args.feedback_timeout_ms <= 5000:
        parser.error('feedback-timeout-ms 必须在 1..5000')
    ports = (args.port, args.http_port, args.camera_http_port, args.camera_port, args.robot_port)
    if len(set(ports)) != 5 or any(not 1 <= p <= 65535 for p in ports):
        parser.error('网页/通信入口及后台端口必须不同且有效')
    if any(not host or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-:'
                           for c in host) for host in args.hosts):
        parser.error('host 只接受 IP/主机名，不接受 URL、路径或凭据')
    if any(n not in ('chassis', 'tracking', 'probe', 'accessory', 'driver', 'motor_register', 'odometry', 'system')
           for n in args.mcap_modules.split(',')) and args.mcap_modules != 'all':
        parser.error('未知 MCAP 模块')
    return args


def sourced(args, command):
    """ROS 进程使用相机依赖后再叠加新底盘接口，避免旧消息遮盖新接口。"""
    script = ('set -e; source /opt/ros/humble/setup.bash; '
              'source "$1/deploy/env.sh" "$2"; '
              'source "$3/install/local_setup.bash"; shift 3; exec "$@"')
    return ['bash', '-c', script, 'robot-session', str(ROOT), args.platform,
            str(args.chassis_workspace.resolve()), *command]


def launch_plan(args, bridge_config):
    command = ['ros2', 'launch', 'duct_robot_bringup', 'hardware.launch.xml',
               f'drive_mode:={args.drive_mode}', 'start_tracking:=true',
               f'tracking_dry_run:={str(not args.real).lower()}', 'start_keyboard:=false',
               f'feedback_timeout_ms:={args.feedback_timeout_ms}', 'start_web_bridge:=true',
               'web_host:=127.0.0.1',
               f'web_port:={args.robot_port}', f'web_bridge_params_file:={bridge_config}',
               f'record_mcap:={str(not args.no_record).lower()}', 'mcap_layout:=modules',
               f'mcap_modules:={args.mcap_modules}',
               f'mcap_output_root:={args.data_root.resolve() / "robot_bags"}']
    if args.tracking_params:
        command.append(f'tracking_params_file:={args.tracking_params.resolve()}')
    plan = [('robot', sourced(args, command))]
    if not args.no_camera:
        camera = ['bash', str(ROOT / 'apps/web_console/run.sh'), '--config',
                  str(Path(bridge_config).parent / 'camera.json')]
        if args.camera_mode == 'd405':
            camera = ['env', f'FASTRTPS_DEFAULT_PROFILES_FILE={ROOT / "config/communication/fastdds_d405_127.xml"}', *camera]
        plan.append(('camera', camera))
    gateway = ['python3', str(ROOT / 'tools/robot_gateway.py'),
                 '--port', str(args.port), '--camera-port', str(args.camera_port),
                 '--robot-port', str(args.robot_port), '--http-port', str(args.http_port),
                 '--camera-http-port', str(args.camera_http_port),
                 '--web-root', str(args.web_root.resolve())]
    for host in args.hosts:
        gateway.extend(['--host', host])
    plan.append(('gateway', sourced(args, gateway)))
    return plan


def bridge_config(args):
    """白名单开放所有指定网页地址和 localhost；Token 不进入配置/日志。"""
    origins = [f'http://{host}:{port}' for host in (*args.hosts, '127.0.0.1', 'localhost')
               for port in (args.http_port, args.camera_http_port)]
    config = {'/**': {'ros__parameters': {
        'host': '127.0.0.1', 'port': args.robot_port,
        'drive_mode': args.drive_mode, 'enable_probe': args.drive_mode == 'tracked_2wd',
        'recording_root': '' if args.no_record else str(args.data_root.resolve() / 'robot_bags'),
        'allowed_origins': list(dict.fromkeys(origins)),
    }}}
    # JSON 是 YAML 子集；无需依赖现场 Python 是否已安装 PyYAML。
    return json.dumps(config, ensure_ascii=False, indent=2)


def camera_config(args):
    """只改变运行副本的后台端口；算法/车型参数不变，不覆盖权威源 YAML。"""
    import yaml
    name = {'all': 'robot_all.yaml', 'd405': 'd405_candidate.yaml',
            'cameras': 'cameras.yaml'}[args.camera_mode]
    try:
        data = yaml.safe_load((ROOT / 'config/communication' / name).read_text())
    except yaml.YAMLError as error:
        raise ValueError('相机源 YAML 无效，未生成运行副本') from error
    if not isinstance(data, dict):
        raise ValueError('相机配置必须是对象')
    data['websocket_port'], data['http_port'] = args.camera_port, args.camera_http_port
    # 配置副本移到临时目录后，已知资源路径保持指向原项目，避免相对路径漂移。
    for section, key, base in (('vision', 'algorithm_config', ROOT / 'config'),
                               ('vision', 'python', ROOT), ('emat', 'saved_waveform', ROOT)):
        value = data.get(section, {}).get(key)
        if isinstance(value, str) and value and not Path(value).is_absolute():
            data[section][key] = str(base / value)
    return json.dumps(data, ensure_ascii=False, indent=2)


def preflight(args):
    errors = []
    for relative in ('index.html', 'console.js', 'connection_config.js', 'robot_embed.js',
                     'robot/index.html', 'robot/robot.js', 'robot/robot.css'):
        if not (args.web_root / relative).is_file():
            errors.append(f'扩展版网页文件缺失：{args.web_root / relative}')
    for path in (Path('/opt/ros/humble/setup.bash'),
                 args.chassis_workspace / 'install/local_setup.bash'):
        if not path.is_file():
            errors.append(f'缺少 {path}')
    if args.tracking_params and not args.tracking_params.is_file():
        errors.append(f'循迹配置不存在：{args.tracking_params}')
    if not args.no_camera:
        backend = ROOT / 'packages/inspection_runtime/src/inspection_runtime/supervisor.py'
        if not backend.is_file() or backend.stat().st_size == 0:
            errors.append('相机工程 supervisor.py 为空/缺失；请先部署完整相机后台，不能用空占位文件启动')
        try:
            camera_config(args)
        except (OSError, ValueError, ImportError) as error:
            errors.append(f'相机运行配置不可用：{error}')
    for port in (args.port, args.http_port, args.robot_port) + (() if args.no_camera else (args.camera_http_port, args.camera_port)):
        try:
            with socket.socket() as check:
                check.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                check.bind(('0.0.0.0', port))
        except OSError as error:
            errors.append(f'端口 {port} 不可绑定（{error}）；请检查已有进程/权限，不会自动杀死其他进程')
    if not errors:
        check = subprocess.run(sourced(args, ['python3', '-c',
            'import rclpy, websockets; from duct_robot_msgs.msg import ControlAuthorityState, WeldTrackingStatus; '
            'from ament_index_python.packages import get_package_share_directory as p; '
            '[p(n) for n in ("duct_robot_bringup","duct_robot_web_bridge","duct_robot_tracking")]']),
            capture_output=True, text=True)
        if check.returncode:
            errors.append('ROS 工作区/新消息未构建或环境混用：' + check.stderr[-1500:])
    return errors


def stop_children(children):
    """只停止本次会话的进程组，先 SIGINT 允许 rosbag2 收尾，不广泛 pkill。"""
    for process in children:
        if process.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGINT)
    deadline = time.monotonic() + 12
    while any(p.poll() is None for p in children) and time.monotonic() < deadline:
        time.sleep(.1)
    for process in children:
        if process.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGTERM)
    for process in children:
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main(argv=None):
    args = arguments(argv)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='0',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', FIELD_DATA_ROOT=str(args.data_root.resolve()),
                      INSPECTION_PLATFORM=args.platform)
    # 原相机配置也是 domain 77；其他域必须显式对齐，不能只改一半。
    if not args.no_camera and args.domain != 77:
        raise SystemExit('整合相机模式当前使用 domain 77；其他域须同步修改相机配置')
    urls = '\n'.join(f'http://{host}:{args.http_port}/' for host in args.hosts)
    if args.plan:
        print('网页（同一服务，可任选可达地址）：\n' + urls)
        print('底盘桥接配置：' + bridge_config(args))
        if not args.no_camera:
            config = json.loads(camera_config(args))
            print(f'传感器模式：{args.camera_mode}；运行副本使用后台端口 {args.camera_port}')
            print('硬件启动配置：' + json.dumps(config['hardware'], ensure_ascii=False))
        for name, command in launch_plan(args, Path('/tmp/robot-session/bridge.json')):
            print(name + ': ' + shlex.join(command))
        return 0
    errors = preflight(args)
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 2
    if args.check:
        print('启动依赖和端口预检通过；未连接设备，未验证硬件安全或实际运动。')
        return 0
    runtime = ROOT / '.runtime' / args.platform
    runtime.mkdir(parents=True, exist_ok=True)
    with (runtime / 'robot-session.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('已有集成会话运行，请在原终端 Ctrl+C 后再启动', file=sys.stderr)
            return 2
        children, logs = [], []
        stopped = False
        def request_stop(signum, frame):
            nonlocal stopped
            stopped = True
        old = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
        with tempfile.TemporaryDirectory(prefix='robot-session-') as temporary:
            config = Path(temporary) / 'bridge.json'
            config.write_text(bridge_config(args))
            if not args.no_camera:
                (Path(temporary) / 'camera.json').write_text(camera_config(args))
            log_root = runtime / 'robot_sessions' / time.strftime('%Y%m%d_%H%M%S')
            log_root.mkdir(parents=True, exist_ok=True)
            print(f'电脑打开（网线/无线任选可达地址）：\n{urls}\n日志：{log_root}\n底盘分模块 MCAP：{args.data_root.resolve() / "robot_bags"}\n'
                  f'循迹：{"实车（需网页人工启用）" if args.real else "预览，不下发到底盘"}\nCtrl+C 停止整套会话并收尾录制。', flush=True)
            result = 0
            try:
                for name, command in launch_plan(args, config):
                    log = (log_root / f'{name}.log').open('w')
                    logs.append(log)
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               stdin=subprocess.DEVNULL, start_new_session=True)
                    children.append(process)
                while not stopped:
                    for (name, _), process in zip(launch_plan(args, config), children):
                        if process.poll() is not None:
                            print(f'{name} 已退出（{process.returncode}），停止其他本次子进程；查看 {log_root}/{name}.log', file=sys.stderr)
                            result = 1
                            stopped = True
                            break
                    time.sleep(.2)
            finally:
                stop_children(children)
                for log in logs:
                    log.close()
                for sig, handler in old.items():
                    signal.signal(sig, handler)
            return result


if __name__ == '__main__':
    raise SystemExit(main())
