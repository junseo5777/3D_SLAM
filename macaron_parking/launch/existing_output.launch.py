"""기존 macaron JOY·명령 선택 파일을 수정 없이 실행하는 선택적 launch입니다.

주차 변환 노드는 별도로 실행합니다. Arduino 펌웨어는 업로드하지 않습니다.
ROS Joy 드라이버와 장치에 맞는 micro-ROS Agent는 기존 방식으로 실행합니다.
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _nodes(context):
    directory = Path(LaunchConfiguration('macaron_dir').perform(context)).expanduser()
    files = [directory / 'teleop_joy.py', directory / 'total_node.py']
    for path in files:
        if not path.is_file():
            raise RuntimeError('기존 macaron 출력 파일을 찾을 수 없습니다: ' + str(path))
    return [ExecuteProcess(cmd=['python3', str(path)], output='screen') for path in files]


def generate_launch_description():
    candidates = [parent / 'src/macaron8_HL_1' for parent in Path(__file__).resolve().parents]
    default = next((str(p) for p in candidates if (p / 'total_node.py').is_file()), '')
    return LaunchDescription([
        DeclareLaunchArgument('macaron_dir', default_value=default,
                              description='기존 teleop_joy.py와 total_node.py가 있는 폴더'),
        OpaqueFunction(function=_nodes),
    ])
