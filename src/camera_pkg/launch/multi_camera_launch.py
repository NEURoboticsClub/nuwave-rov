from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from cv2_enumerate_cameras import enumerate_cameras
import cv2


DEFAULT_PORTS = [0, 4, 8, 12]


def _path_to_index(path):
    marker = '/dev/video'
    if not isinstance(path, str) or not path.startswith(marker):
        return None
    suffix = path[len(marker):]
    if suffix.isdigit():
        return int(suffix)
    return None


def _is_working_path(path):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return False

    read, _ = cap.read()
    cap.release()
    return bool(read)


def _enumerated_cameras():
    try:
        return list(enumerate_cameras())
    except Exception:
        return []


def _discover_working_ports(max_cameras=4):
    # Group by camera display name so each physical camera can try all of 
    # its enumerated paths, then keep the first readable one.
    groups = {}
    for cam_idx, cam in enumerate(_enumerated_cameras()):
        name = getattr(cam, 'name', None)
        group_key = name.strip() if isinstance(name, str) and name.strip() else f'camera_{cam_idx}'
        path = getattr(cam, 'path', None)
        index = _path_to_index(path)
        if index is None:
            raw_index = getattr(cam, 'index', None)
            if isinstance(raw_index, int):
                index = raw_index

        if index is None:
            continue

        groups.setdefault(group_key, [])
        if index not in groups[group_key]:
            groups[group_key].append(index)

    selected = []
    for _, candidates in groups.items():
        for index in candidates:
            path = f'/dev/video{index}'
            if _is_working_path(path):
                selected.append(index)
                break
        if len(selected) >= max_cameras:
            return selected

    # Fallback to default ports if enumeration fails or finds no working cameras
    if not selected:
        selected = DEFAULT_PORTS.copy()

    return selected[:max_cameras]


def generate_launch_description():
    camera_ports = _discover_working_ports(max_cameras=4)

    cam0_enabled = LaunchConfiguration('cam0_enabled')
    cam1_enabled = LaunchConfiguration('cam1_enabled')
    cam2_enabled = LaunchConfiguration('cam2_enabled')
    cam3_enabled = LaunchConfiguration('cam3_enabled')

    actions = [
        DeclareLaunchArgument('cam0_enabled', default_value='true'),
        DeclareLaunchArgument('cam1_enabled', default_value='true'),
        DeclareLaunchArgument('cam2_enabled', default_value='true'),
        DeclareLaunchArgument('cam3_enabled', default_value='true'),
    ]

    enabled_flags = [cam0_enabled, cam1_enabled, cam2_enabled, cam3_enabled]

    for camera_id, camera_port in enumerate(camera_ports):
        actions.append(
            Node(
                package='camera_pkg',
                executable='camera_publisher',
                name=f'camera_publisher_{camera_id}',
                output='screen',
                parameters=[
                    {'camera_id': camera_id},
                    {'camera_port': camera_port},
                    {'camera_device': f'/dev/video{camera_port}'},
                    {'width': 320},
                    {'height': 240},
                    {'fps': 30},
                    {'jpeg_quality': 70},
                ],
                condition=IfCondition(enabled_flags[camera_id]),
            )
        )

    return LaunchDescription(actions)
