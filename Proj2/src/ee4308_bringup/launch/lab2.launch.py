from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

import xacro
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros') 
    pkg_ee4308_bringup = get_package_share_directory('ee4308_bringup')

    turtle_ns = 'turtle'
    drone_ns = 'drone' # overwritten below
    drone_yaml = os.path.join(pkg_ee4308_bringup, 'params', 'drone.yaml')
    # get drone nss
    with open(drone_yaml, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        drone_ns = yaml_dict['namespace'] # make sure it is 'drone'

    drone_x = LaunchConfiguration('drone_x')
    drone_x_arg = DeclareLaunchArgument('drone_x', default_value='0.0')
    ld.add_action(drone_x_arg)

    drone_y = LaunchConfiguration('drone_y')
    drone_y_arg = DeclareLaunchArgument('drone_y', default_value='-2.0')
    ld.add_action(drone_y_arg)

    drone_z = LaunchConfiguration('drone_z')
    drone_z_arg = DeclareLaunchArgument('drone_z', default_value='0.05')
    ld.add_action(drone_z_arg)
    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value=os.path.join(pkg_ee4308_bringup, 'worlds', 'turtlebot3_house.world'))
    ld.add_action(world_arg) # note the worlds must have a more relaxed contact coefficient (~100), and a less accurate solver to prevent erroneous collisions resulting in NAN values.
    
    gz_client = LaunchConfiguration('gz_client') # if true, gz_client is not launched
    gz_client = DeclareLaunchArgument('gz_client', default_value='False')
    ld.add_action(gz_client) 
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    ld.add_action(use_sim_time_arg)

    yaml_filename = LaunchConfiguration('yaml_filename')
    yaml_filename_arg = DeclareLaunchArgument('yaml_filename', default_value=os.path.join(pkg_ee4308_bringup, 'maps', 'my_map_sim.yaml'))
    ld.add_action(yaml_filename_arg)
    
    
    # get drone description
    drone_description_config = xacro.process_file(
        os.path.join(pkg_ee4308_bringup, 'urdf', 'drone.urdf.xacro'), 
        mappings={'params_path': drone_yaml})
    drone_desc = drone_description_config.toxml()

    # get proj2.yaml
    params_yaml = os.path.join(pkg_ee4308_bringup, 'params', 'proj2.yaml')

    # open gazebo server
    launch_gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world,
                          'verbose': 'true',
                          }.items()
    )
    ld.add_action(launch_gzserver)

    # open gazebo client
    launch_gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gz_client')),
        launch_arguments={'verbose': 'true'}.items(),
    )
    ld.add_action(launch_gzclient)

    # drone state publisher
    node_drone_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=drone_ns,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': drone_desc, 'frame_prefix': drone_ns + '/'}],
        arguments=[drone_desc]
    )
    ld.add_action(node_drone_state_publisher)


    # static transform for drone
    node_drone_static_tf = Node(
        package='tf2_ros',
        name='drone_static_tf',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', f'{drone_ns}/odom'],
        output='screen'
    )
    ld.add_action(node_drone_static_tf)

    node_turtle_map_server = Node(
        namespace=turtle_ns,
        package='ee4308_turtle2',
        executable='map_server',
        parameters=[params_yaml],
        arguments=[yaml_filename],
        output='screen',
        emulate_tty=True,
    )

    node_drone_estimator = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        executable='estimator',
        parameters=[params_yaml],
        arguments=[drone_x, drone_y, drone_z],
        output='screen',
        emulate_tty=True,
    )

    node_drone_planner = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        executable='planner',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )

    node_drone_controller = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        name='controller_ee4308',
        executable='controller',
        parameters=[params_yaml],
        output='screen',
        emulate_tty=True,
    )

    node_drone_behavior = Node(
        namespace=drone_ns,
        package='ee4308_drone',
        executable='behavior',
        parameters=[params_yaml],
        arguments=[drone_x, drone_y, drone_z],
        output='screen',
        emulate_tty=True,
    )

    # rviz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ee4308_bringup, 'rviz', 'proj2.rviz'),],
        output='screen'
    )

    # spawn drone
    node_spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', drone_ns, # name of robot
            '-topic', f'{drone_ns}/robot_description',
            '-x', drone_x,
            '-y', drone_y,
            '-z', drone_z,
            '-robot_namespace', drone_ns
        ],
        output='screen',
        on_exit=[node_rviz, node_turtle_map_server, node_drone_estimator, node_drone_planner, node_drone_controller, node_drone_behavior] # spawn the drone first bcos it may collide with the turtle. (note tbot is at origin, spawner spawns everything at origin before moving it to 'initial pose')
    )
    ld.add_action(node_spawn_drone)

    return ld