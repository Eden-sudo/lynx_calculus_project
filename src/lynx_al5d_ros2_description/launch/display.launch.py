import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'lynx_al5d_ros2_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Ruta al archivo Xacro
    default_model_path = os.path.join(pkg_share, 'urdf', 'lynx_al5d.urdf.xacro')
    
    # Ruta a la configuracion de RViz (opcional, usaremos la default por ahora)
    # default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    # Convertir Xacro a URDF en tiempo de ejecucion
    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    
    # 1. NODO: Robot State Publisher (Publica la estructura del robot)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 2. NODO: Joint State Publisher GUI (Ventana con sliders para mover el robot)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 3. NODO: RViz2 (El visualizador 3D)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', default_rviz_config_path] # Descomentar si tienes config guardada
    )

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
    	output='screen',
    	arguments=['-d', rviz_config_file] # Esto carga tu layout guardado
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', 
            default_value=default_model_path, 
            description='Absolute path to robot urdf file'),
        
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
