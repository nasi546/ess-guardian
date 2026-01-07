import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 경로 설정
    nav2_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
    map_file = os.path.expanduser('~/2512_4th_proj_chanmi/ess_map/ess_map.yaml')
    param_file = os.path.expanduser('~/2512_4th_proj_chanmi/src/ess_bringup/params/my_burger.yaml')
    
    # 로그를 저장할 파일 경로 (홈 디렉토리의 control_log.txt)
    log_file_path = os.path.expanduser('~/control_log.txt')

    # (옵션) 시작할 때 기존 로그 파일을 비우고 싶다면 아래 두 줄의 주석을 푸세요
    # with open(log_file_path, 'w') as f:
    #     f.write("")

    # 2. Control Node 정의 (변수에 담아둡니다)
    # output='screen'을 지우면 화면에는 안 나오고 파일에만 저장됩니다.
    control_node = Node(
        package='ess_control_pkg',
        executable='control_node',
        output='screen' 
    )

    # 3. 로그 저장 핸들러 정의
    # control_node에서 글자(stdout)가 나올 때마다 파일에 이어씁니다('a').
    save_log_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=control_node,
            on_stdout=lambda event: open(log_file_path, 'a').write(event.text.decode())
        )
    )

    return LaunchDescription([
        # 1. Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file,
                'params_file': param_file,
                'use_rviz': 'False',
                'initial_pose_x': '0.0',
                'initial_pose_y': '0.0',
                'initial_pose_a': '0.0'
            }.items()
        ),

        # 2. 초기 위치 (Initial Pose)
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--once', '/initialpose', 
                         'geometry_msgs/msg/PoseWithCovarianceStamped', 
                         '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'],
                    output='screen'
                )
            ]
        ),

        # 3. MQTT Bridge
        Node(
            package='ess_mqtt_bridge_pkg',
            executable='mqtt_bridge_node',
            output='screen'
        ),

        # 4. Control Node 실행
        control_node,

        # 5. 로그 저장 기능 실행 (Control Node 감시)
        save_log_handler
    ])