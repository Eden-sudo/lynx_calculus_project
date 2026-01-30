import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'lynx_al5d_ros2_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 1. Configurar rutas para modelos (evita mallas invisibles)
    gazebo_model_path = os.path.join(pkg_share, '..')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_model_path

    # 2. Procesar el URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'lynx_al5d.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # 3. Nodo Robot State Publisher (Publica TF basado en la simulación)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 4. Lanzar Gazebo
    # Truco para tu laptop: Si quieres ahorrar recursos, puedes cambiar 'gui': True a False
    # y solo verás el robot en RViz, pero la física correrá de fondo.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 5. Spawn del robot en Gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'lynx_al5d'],
        output='screen'
    )

    # 6. ACTIVAR CONTROLADORES
    # Carga el publicador de estados (lee los encoders simulados)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Carga el controlador del brazo (recibe comandos de posición)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    # Retrasar la carga de controladores hasta que el robot haya aparecido (Spawn)
    # Esto evita errores de "Controller manager not found" al inicio
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner],
            )
        ),
    ])
