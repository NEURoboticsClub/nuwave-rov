from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from cv2_enumerate_cameras import enumerate_cameras
import cv2


def _is_working_path(path):
    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return False

    read, _ = cap.read()
    cap.release()
    return bool(read)


def _discover_working_ports():
    # Group by camera display name so each physical camera can try all of 
    # its enumerated paths, then keep the first readable one.
    idx_list = []
    for cam in enumerate_cameras():
        vid = getattr(cam, 'vid', None)
        pid = getattr(cam, 'pid', None)
        if (vid == 3141 and pid == 25446):
            index = getattr(cam, 'index', None)
            if index % 100 not in idx_list:
                idx_list.append(index % 100)
    
    print("index list: ", idx_list)

    camera_list = []
    
    for candidate in idx_list:
        path = f'/dev/video{candidate}'
        if _is_working_path(path):
            camera_list.append(candidate)

    return camera_list


def generate_launch_description():
    # Raising from generate_launch_description() makes the launch frontend dump
    # unrelated tracebacks (InvalidFrontendLaunchFileError), so on failure we
    # return a single clear log message instead.
    try:
        camera_ports = _discover_working_ports()
    except Exception as e:
        msg = f'Camera discovery failed ({e}); shutting down camera launch.'
        return LaunchDescription([
            LogInfo(msg=msg),
        ])

    if not camera_ports:
        msg = 'No working cameras found; shutting down camera launch.'
        return LaunchDescription([
            LogInfo(msg=msg),
        ])

    actions = []

    enabled_flags = []

    for camera_id, camera_port in enumerate(camera_ports):
        enabled_flags.append(LaunchConfiguration(f'cam{camera_id}_enabled'))
        actions.append(DeclareLaunchArgument(f'cam{camera_id}_enabled', default_value='true'))
        actions.append(
            Node(
                package='camera_pkg',
                executable='camera_publisher',
                name=f'camera_publisher_{camera_id}',
                output='screen',
                parameters=[
                    {'camera_id': camera_id},
                    {'camera_port': camera_port},
                    {'camera_device_path': f'/dev/video{camera_port}'},
                    {'width': 320},
                    {'height': 240},
                    {'fps': 30},
                    {'jpeg_quality': 70},
                ],
                condition=IfCondition(enabled_flags[camera_id]),
            )
        )

    return LaunchDescription(actions)
