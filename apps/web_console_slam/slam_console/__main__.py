"""CLI, static preflight and configuration resolution. --plan never imports ROS."""
import argparse
import asyncio
import importlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import sys

import yaml

APP = Path(__file__).resolve().parents[1]


def arguments(argv=None):
    p = argparse.ArgumentParser(description='独立定位与建图服务，HTTP/WS :8082；输入由外部提供')
    p.add_argument('--platform', choices=('s100p', 'rk3588', 'x86'), default=os.environ.get('INSPECTION_PLATFORM', 's100p'))
    p.add_argument('--domain', type=int, default=77)
    p.add_argument('--host', action='append')
    p.add_argument('--config', type=Path, default=APP / 'config/slam.yaml')
    p.add_argument('--plan', action='store_true')
    p.add_argument('--check', action='store_true')
    p.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    a = p.parse_args(argv)
    a.hosts = list(dict.fromkeys(a.host or ['192.168.127.10', '192.168.1.148', 'localhost', '127.0.0.1']))
    if not 0 <= a.domain <= 232:
        p.error('domain 必须为 0..232')
    if any(not h or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-:' for c in h) for h in a.hosts):
        p.error('host 只接受 IP/主机名')
    return a


def load_config(path, platform):
    path = path.resolve()
    c = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(c, dict):
        raise ValueError('配置须为对象')
    def resolve(value, base=path.parent):
        return (base / value).resolve()
    c['algorithm_root'] = resolve(c['algorithm_root'])
    c['prior_directory'] = resolve(c.get('prior_directory', 'PCD/map'), c['algorithm_root'])
    c['runtime_directory'] = resolve(c['runtime_directory'].format(platform=platform))
    if c.get('base_config', 'installed') == 'installed':
        from ament_index_python.packages import get_package_share_directory
        c['base_config'] = Path(get_package_share_directory('fast_livo')) / 'config/mapping_relocation.yaml'
    else:
        c['base_config'] = resolve(c['base_config'])
    for key, default in [('input_max_age', 2), ('stamp_tolerance', 3), ('mapping_timeout', 60), ('relocation_timeout', 180), ('shutdown_timeout', 130)]:
        c[key] = float(c.get(key, default))
        if not 0 < c[key] < 86400:
            raise ValueError(f'{key} 超出范围')
    if c['shutdown_timeout'] < 130 or c['shutdown_timeout'] > 140:
        raise ValueError('shutdown_timeout 须在 130..140 秒内，整套服务预算为 150 秒')
    for key in ('lidar_topic', 'imu_topic'):
        if not isinstance(c.get(key), str) or not c[key].startswith('/'):
            raise ValueError(f'{key} 须为绝对 ROS 话题')
    if c.get('imu_acceleration_unit') != 'm/s2':
        raise ValueError('必须配置并提供米制 IMU：imu_acceleration_unit: m/s2')
    base_parameters(c)
    return c


def base_parameters(config):
    data = yaml.safe_load(config['base_config'].read_text(encoding='utf-8'))
    try:
        p = data['/**']['ros__parameters']
    except (KeyError, TypeError) as error:
        raise ValueError('基础 YAML 缺少 /**/ros__parameters') from error
    if not isinstance(p, dict):
        raise ValueError('基础 YAML 缺少 /**/ros__parameters')
    return data


def static_check(c, check_port=True):
    errors = []
    for module in ('numpy', 'yaml', 'websockets.asyncio.server', 'rclpy', 'sensor_msgs.msg', 'nav_msgs.msg', 'geometry_msgs.msg', 'std_msgs.msg', 'livox_ros_driver2.msg'):
        try:
            importlib.import_module(module)
        except (ImportError, OSError) as error:
            errors.append(f'依赖 {module} 不可用：{error}')
    try:
        from ament_index_python.packages import get_package_share_directory, get_package_prefix
        for package, launch, executable in [('fast_livo', 'mapping_relocation.launch.py', 'fastlivo_mapping'), ('relocalization', 'relocalization.launch.py', 'teaser_gicp_node')]:
            for path in (Path(get_package_share_directory(package)) / 'launch' / launch,
                         Path(get_package_prefix(package)) / 'lib' / package / executable):
                if not path.is_file():
                    errors.append(f'算法未完整安装：{path}')
        if not shutil.which('ros2'):
            errors.append('缺少 ros2 命令')
    except (ImportError, LookupError) as error:
        errors.append(f'算法安装不可用：{error}')
    if not c['algorithm_root'].is_dir():
        errors.append(f'算法根目录不存在：{c["algorithm_root"]}')
    if check_port:
        try:
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(('0.0.0.0', 8082))
        except OSError as error:
            errors.append(f'端口 8082 不可绑定：{error}；不会接管现有服务')
    return errors


def main(argv=None):
    a = arguments(argv)
    os.environ.update(ROS_DOMAIN_ID=str(a.domain), ROS_LOCALHOST_ONLY='0')
    if a.plan:
        print(json.dumps({'entry': 'python3 -m slam_console', 'config': str(a.config.resolve()),
                          'platform': a.platform, 'domain': a.domain, 'hosts': a.hosts,
                          'http_ws_port': 8082, 'initial_state': 'idle', 'sensors': 'external'}, ensure_ascii=False, indent=2))
        return 0
    try:
        c = load_config(a.config, a.platform)
        errors = static_check(c)
        if errors:
            raise ValueError('\n'.join(errors))
        if a.check:
            print('SLAM 软件、配置和端口预检通过；未启动硬件，实时输入在点击启动时检查。')
            return 0
        if not a.worker:
            from .session import guard
            # Keep stdin open for terminal and supervisor launches; parent signals this wrapper.
            # Service grandchildren remain adopted and are awaited even after service crashes.
            return guard([sys.executable, '-m', 'slam_console', '--worker', *(argv if argv is not None else sys.argv[1:])], 140, watch_stdin=False, death_signal=signal.SIGKILL)
        from .server import run
        asyncio.run(run(c, a.hosts))
        return 0
    except (ValueError, KeyError, TypeError, OSError, ImportError, LookupError, yaml.YAMLError) as error:
        print(f'SLAM：{error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
