"""YAML-configured D400 startup, including ROS 2 Foxy RealSense driver APIs."""
import ast
import os
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_config(path):
    with path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not config:
        raise ValueError('Expected a non-empty YAML mapping: ' + str(path))
    blocks = [(key, value['ros__parameters']) for key, value in config.items()
              if isinstance(value, dict) and 'ros__parameters' in value]
    if blocks:
        if len(config) != 1 or len(blocks) != 1:
            raise ValueError('Use a YAML file containing exactly one camera node.')
        node_path, config = blocks[0]
        if not isinstance(config, dict):
            raise ValueError('ros__parameters must contain a mapping.')
        config = dict(config)
        if node_path != '/**':
            parts = node_path.strip('/').split('/')
            config.setdefault('camera_name', parts[-1])
            config.setdefault('camera_namespace', '/'.join(parts[:-1]))
    return _flatten(config)


def _flatten(config, prefix=''):
    result = {}
    for key, value in config.items():
        if not isinstance(key, str):
            raise ValueError('Parameter keys must be strings')
        name = prefix + key
        items = (_flatten(value, name + '.') if isinstance(value, dict)
                 else {name: value})
        for flat_key, flat_value in items.items():
            if flat_key in result:
                raise ValueError('Duplicate parameter: ' + flat_key)
            result[flat_key] = flat_value
    return result


def _detect_api(share):
    """Inspect declarations, never execute the installed driver's launch code."""
    launch_file = share / 'launch' / 'rs_launch.py'
    if launch_file.is_file():
        tree = ast.parse(launch_file.read_text(encoding='utf-8'))
        names = set()
        for item in ast.walk(tree):
            if isinstance(item, ast.Dict):
                for key, value in zip(item.keys, item.values):
                    if (isinstance(key, ast.Constant) and key.value == 'name'
                            and isinstance(value, ast.Constant)):
                        names.add(value.value)
        if 'depth_module.depth_profile' in names:
            return 'split'
        if 'depth_module.profile' in names:
            return 'profile'
        if 'depth_width' in names:
            return 'legacy'
    version = ET.parse(str(share / 'package.xml')).findtext('version', '')
    if version.startswith('3.'):
        return 'legacy'
    raise ValueError('Cannot detect RealSense parameter API (version ' + version
                     + '); set driver_api:=legacy, profile or split explicitly.')


def _profile(value):
    if not isinstance(value, str):
        raise ValueError('Image profiles must be quoted strings, e.g. "640x480x30"')
    fields = re.split(r'\s*[xX,]\s*', value.strip())
    if len(fields) != 3 or not all(field.isdigit() for field in fields):
        raise ValueError('Invalid WIDTHxHEIGHTxFPS profile: ' + value)
    result = tuple(int(field) for field in fields)
    if min(result) <= 0:
        raise ValueError('Image profile values must be positive: ' + value)
    return result


def _translate(config, api):
    """Translate the main branch's split-profile YAML to older driver schemas."""
    if api not in ('legacy', 'profile', 'split'):
        raise ValueError('driver_api must be auto, legacy, profile or split')
    params = dict(config)
    notes = []

    def put(key, value):
        if key in params and params[key] != value:
            raise ValueError('Conflicting settings for ' + key)
        params[key] = value

    if api == 'split':
        return params, notes
    # These releases use node name/namespace, not a camera_name ROS parameter.
    params.pop('camera_name', None)
    for name in ('enable_rgbd', 'enable_infra'):
        if params.pop(name, False):
            raise ValueError(name + ' is unsupported by this older driver API')
    for key, expected in [('depth_module.depth_format', 'Z16'),
                          ('depth_module.infra1_format', 'Y8'),
                          ('depth_module.infra2_format', 'Y8'),
                          ('rgb_camera.color_format', 'RGB8')]:
        if key in params:
            value = params.pop(key)
            if value != expected:
                raise ValueError(key + ': older drivers require the standard ' + expected + ' format')
    if api == 'profile':
        profiles = []
        for key, active in [('depth_module.depth_profile', params.get('enable_depth', True)),
                            ('depth_module.infra_profile', params.get('enable_infra1', False)
                             or params.get('enable_infra2', False))]:
            if key in params:
                value = _profile(params.pop(key))
                if active:
                    profiles.append(value)
        if profiles:
            if len(set(profiles)) != 1:
                raise ValueError('This driver shares depth_module.profile: enabled depth/infra profiles must match')
            put('depth_module.profile', ','.join(str(v) for v in profiles[0]))
        if 'rgb_camera.color_profile' in params:
            put('rgb_camera.profile', ','.join(str(v) for v in _profile(params.pop('rgb_camera.color_profile'))))
    else:
        for source, stream in [('depth_module.depth_profile', 'depth'),
                               ('depth_module.infra_profile', 'infra'),
                               ('rgb_camera.color_profile', 'color')]:
            if source in params:
                width, height, fps = _profile(params.pop(source))
                put(stream + '_width', width)
                put(stream + '_height', height)
                put(stream + '_fps', float(fps))
        for key in list(params):
            if key.startswith('depth_module.'):
                put('stereo_module.' + key.split('.', 1)[1], params.pop(key))
        qos = [params.pop(key) for key in ('infra1_qos', 'infra2_qos') if key in params]
        if qos:
            if len(set(qos)) != 1:
                raise ValueError('Legacy drivers share infra_qos between the two infrared cameras')
            put('infra_qos', qos[0])
        renames = {'pointcloud.enable': 'enable_pointcloud',
                   'pointcloud.stream_index_filter': 'pointcloud_texture_index',
                   'align_depth.enable': 'align_depth'}
        for source, target in renames.items():
            if source in params:
                put(target, params.pop(source))
        if 'pointcloud.stream_filter' in params:
            streams = {0: 'RS2_STREAM_ANY', 1: 'RS2_STREAM_DEPTH',
                       2: 'RS2_STREAM_COLOR', 3: 'RS2_STREAM_INFRARED', 4: 'RS2_STREAM_FISHEYE'}
            value = params.pop('pointcloud.stream_filter')
            if value not in streams:
                raise ValueError('Unsupported legacy pointcloud texture stream: ' + str(value))
            put('pointcloud_texture_stream', streams[value])
        filters = []
        for key, name in [('decimation_filter.enable', 'decimation'),
                          ('spatial_filter.enable', 'spatial'), ('temporal_filter.enable', 'temporal'),
                          ('disparity_filter.enable', 'disparity'), ('hole_filling_filter.enable', 'hole_filling'),
                          ('hdr_merge.enable', 'hdr_merge'), ('colorizer.enable', 'colorizer')]:
            if params.pop(key, False):
                filters.append(name)
        if filters or 'filters' not in params:
            put('filters', ','.join(filters))
    for source, target in [('pointcloud.allow_no_texture_points', 'allow_no_texture_points'),
                           ('pointcloud.ordered_pc', 'ordered_pc')]:
        if source in params:
            put(target, params.pop(source))
    notes.append('Translated split-profile configuration to RealSense ' + api + ' API')
    return params, notes


def _write_parameters(config):
    # A ROS parameter file preserves empty/numeric-looking strings on Foxy,
    # without relying on Humble's ParameterValue(value_type=str) API.
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                     prefix='d435-params-', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump({'/**': {'ros__parameters': config}}, stream, sort_keys=False)
        return Path(stream.name)


def _launch_camera(context):
    filename = LaunchConfiguration('config_file').perform(context)
    path = Path(os.path.expandvars(filename)).expanduser().resolve()
    config = _read_config(path)
    namespace = config.pop('camera_namespace', 'camera')
    log_level = config.pop('log_level', 'info')
    output = config.pop('output', 'screen')
    name = config.setdefault('camera_name', 'camera')
    if not all(isinstance(v, str) for v in (namespace, log_level, output, name)):
        raise ValueError('camera_name, camera_namespace, log_level and output must be strings')
    for key in ('serial_no', 'usb_port_id', 'device_type', 'json_file_path'):
        if key in config and not isinstance(config[key], str):
            raise ValueError(key + ' must be a quoted YAML string')
    requested = LaunchConfiguration('driver_api').perform(context)
    configured = config.pop('driver_api', 'auto')
    api = configured if requested == 'auto' else requested
    if api == 'auto':
        api = _detect_api(Path(get_package_share_directory('realsense2_camera')))
    params, notes = _translate(config, api)
    parameter_file = _write_parameters(params)

    def cleanup(_context):
        try:
            parameter_file.unlink()
        except FileNotFoundError:
            pass
        return []

    try:
        camera = Node(package='realsense2_camera', executable='realsense2_camera_node',
                      name=name, namespace=namespace, output=output, emulate_tty=True,
                      parameters=[str(parameter_file)],
                      arguments=['--ros-args', '--log-level', log_level])
    except Exception:
        cleanup(context)
        raise
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            LogInfo(msg='D435 configuration: ' + str(path) + '; driver API: ' + api),
            *[LogInfo(msg=note) for note in notes], camera]


def generate_launch_description():
    default = Path(__file__).resolve().parent.parent / 'config' / 'd435.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(default),
                              description='Path to the camera YAML configuration file'),
        DeclareLaunchArgument('driver_api', default_value='auto',
                              description='auto, legacy (3.x), profile (4.51.x), split (newer 4.x)'),
        OpaqueFunction(function=_launch_camera),
    ])
