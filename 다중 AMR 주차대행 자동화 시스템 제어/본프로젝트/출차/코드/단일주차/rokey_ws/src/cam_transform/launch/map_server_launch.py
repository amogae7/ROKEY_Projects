from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import OnStateTransition
from lifecycle_msgs.msg import Transition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 현재 패키지의 쉐어 디렉토리 경로를 가져옵니다.
    pkg_dir = get_package_share_directory('cam_transform')
    
    # teleop_map.yaml 파일 경로 설정
    # (yaml 파일이 launch 파일과 같은 레벨에 있다고 가정합니다. 필요시 경로를 수정하세요)
    map_file_path = os.path.join(pkg_dir, 'teleop_map.yaml') 

    # 1. 맵 서버 라이프사이클 노드 정의
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file_path},
            {'use_sim_time': False}
        ],
        namespace=''
    )

    # 2. 'unconfigured' 상태에서 시작할 때 'configure'를 요청하는 이벤트 핸들러
    configure_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            start_state='unconfigured',
            goal_state='inactive',
            entities=[
                EmitEvent(event=Transition(id=Transition.TRANSITION_CONFIGURE)),
            ],
        )
    )

    # 3. 'inactive' 상태가 되었을 때 'activate'를 요청하는 이벤트 핸들러 (맵 발행 시작)
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            start_state='inactive',
            goal_state='active',
            entities=[
                EmitEvent(event=Transition(id=Transition.TRANSITION_ACTIVATE)),
            ],
        )
    )

    return LaunchDescription([
        map_server_node,
        configure_handler,
        activate_handler
    ])
