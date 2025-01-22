# ================================================================
# from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
# ================================================================

import os
# import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument #, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration #, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_dual_ros2_control_params_temp_file

# ================================================================
# from: xarm_controller/launch/_dual_ros2_control.launch.py
# ================================================================

from launch.launch_description_sources import load_python_launch_file_as_module

# ================================================================
# from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
# ================================================================

def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    dof_1 = LaunchConfiguration('dof_1', default=dof)
    dof_2 = LaunchConfiguration('dof_2', default=dof)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    robot_type_1 = LaunchConfiguration('robot_type_1', default='lite')#default=robot_type)
    robot_type_2 = LaunchConfiguration('robot_type_2', default='uf850')#default=robot_type)
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    model1300_1 = LaunchConfiguration('model1300_1', default=model1300)
    model1300_2 = LaunchConfiguration('model1300_2', default=model1300)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    robot_sn_1 = LaunchConfiguration('robot_sn_1', default=robot_sn)
    robot_sn_2 = LaunchConfiguration('robot_sn_2', default=robot_sn)
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    kinematics_suffix_1 = LaunchConfiguration('kinematics_suffix_1', default=kinematics_suffix)
    kinematics_suffix_2 = LaunchConfiguration('kinematics_suffix_2', default=kinematics_suffix)

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1 = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2 = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1 = LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2 = LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1 = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2 = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=True)
    add_realsense_d435i_1 = LaunchConfiguration('add_realsense_d435i_1', default=add_realsense_d435i)
    add_realsense_d435i_2 = LaunchConfiguration('add_realsense_d435i_2', default=add_realsense_d435i)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_d435i_links_1 = LaunchConfiguration('add_d435i_links_1', default=add_d435i_links)
    add_d435i_links_2 = LaunchConfiguration('add_d435i_links_2', default=add_d435i_links)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=True)
    add_other_geometry_1 = LaunchConfiguration('add_other_geometry_1', default=add_other_geometry)
    add_other_geometry_2 = LaunchConfiguration('add_other_geometry_2', default=add_other_geometry)
    geometry_type = LaunchConfiguration('geometry_type', default='mesh')
    geometry_type_1 = LaunchConfiguration('geometry_type_1', default=geometry_type)
    geometry_type_2 = LaunchConfiguration('geometry_type_2', default=geometry_type)
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.3)
    geometry_mass_1 = LaunchConfiguration('geometry_mass_1', default=geometry_mass)
    geometry_mass_2 = LaunchConfiguration('geometry_mass_2', default=geometry_mass)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_height_1 = LaunchConfiguration('geometry_height_1', default=geometry_height)
    geometry_height_2 = LaunchConfiguration('geometry_height_2', default=geometry_height)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_radius_1 = LaunchConfiguration('geometry_radius_1', default=geometry_radius)
    geometry_radius_2 = LaunchConfiguration('geometry_radius_2', default=geometry_radius)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_length_1 = LaunchConfiguration('geometry_length_1', default=geometry_length)
    geometry_length_2 = LaunchConfiguration('geometry_length_2', default=geometry_length)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_width_1 = LaunchConfiguration('geometry_width_1', default=geometry_width)
    geometry_width_2 = LaunchConfiguration('geometry_width_2', default=geometry_width)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='pneumatic_lite.stl')
    geometry_mesh_filename_1 = LaunchConfiguration('geometry_mesh_filename_1', default=geometry_mesh_filename)
    geometry_mesh_filename_2 = LaunchConfiguration('geometry_mesh_filename_2', default=geometry_mesh_filename)
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_xyz_1 = LaunchConfiguration('geometry_mesh_origin_xyz_1', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_xyz_2 = LaunchConfiguration('geometry_mesh_origin_xyz_2', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_origin_rpy_1 = LaunchConfiguration('geometry_mesh_origin_rpy_1', default=geometry_mesh_origin_rpy)
    geometry_mesh_origin_rpy_2 = LaunchConfiguration('geometry_mesh_origin_rpy_2', default=geometry_mesh_origin_rpy)
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0.03075 0 0.11885"')
    geometry_mesh_tcp_xyz_1 = LaunchConfiguration('geometry_mesh_tcp_xyz_1', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_xyz_2 = LaunchConfiguration('geometry_mesh_tcp_xyz_2', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0.52 0"')
    geometry_mesh_tcp_rpy_1 = LaunchConfiguration('geometry_mesh_tcp_rpy_1', default=geometry_mesh_tcp_rpy)
    geometry_mesh_tcp_rpy_2 = LaunchConfiguration('geometry_mesh_tcp_rpy_2', default=geometry_mesh_tcp_rpy)

    # ================================================================
    # from: src/manymove_planner/launch/lite_micpp_fake_action_server.launch.py
    # ================================================================

    velocity_scaling_factor = LaunchConfiguration('velocity_scaling_factor')
    acceleration_scaling_factor = LaunchConfiguration('acceleration_scaling_factor')
    max_exec_retries = LaunchConfiguration('max_exec_retries')
    smoothing_type = LaunchConfiguration('smoothing_type')
    step_size = LaunchConfiguration('step_size')
    jump_threshold = LaunchConfiguration('jump_threshold')
    max_cartesian_speed = LaunchConfiguration('max_cartesian_speed')
    plan_number_target = LaunchConfiguration('plan_number_target')
    plan_number_limit = LaunchConfiguration('plan_number_limit')

    base_frame = LaunchConfiguration('base_frame')
    tcp_frame = LaunchConfiguration('tcp_frame')

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
    # ================================================================

    # no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware'
    controllers_name = 'fake_controllers'
    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')

    ros2_control_params = generate_dual_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_1)),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_2)),
        prefix_1=prefix_1.perform(context), 
        prefix_2=prefix_2.perform(context), 
        add_gripper_1=add_gripper_1.perform(context) in ('True', 'true'),
        add_gripper_2=add_gripper_2.perform(context) in ('True', 'true'),
        add_bio_gripper_1=add_bio_gripper_1.perform(context) in ('True', 'true'),
        add_bio_gripper_2=add_bio_gripper_2.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type_1=robot_type_1.perform(context), 
        robot_type_2=robot_type_2.perform(context), 
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py

    # WARNING: MODIFIED!!!

    # Grouped DualMoveItConfigsBuilder to apply the following to enable moveitcpp pipelines:

    # .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    # .planning_pipelines(pipelines=["ompl"])
    # .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp.yaml")

    # ================================================================

    moveit_config = (
        DualMoveItConfigsBuilder(
            context=context,
            controllers_name=controllers_name,
            dof_1=dof_1,
            dof_2=dof_2,
            robot_type_1=robot_type_1,
            robot_type_2=robot_type_2,
            prefix_1=prefix_1,
            prefix_2=prefix_2,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300_1=model1300_1,
            model1300_2=model1300_2,
            robot_sn_1=robot_sn_1,
            robot_sn_2=robot_sn_2,
            mesh_suffix=mesh_suffix,
            kinematics_suffix_1=kinematics_suffix_1,
            kinematics_suffix_2=kinematics_suffix_2,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper_1=add_gripper_1,
            add_gripper_2=add_gripper_2,
            add_vacuum_gripper_1=add_vacuum_gripper_1,
            add_vacuum_gripper_2=add_vacuum_gripper_2,
            add_bio_gripper_1=add_bio_gripper_1,
            add_bio_gripper_2=add_bio_gripper_2,
            add_realsense_d435i_1=add_realsense_d435i_1,
            add_realsense_d435i_2=add_realsense_d435i_2,
            add_d435i_links_1=add_d435i_links_1,
            add_d435i_links_2=add_d435i_links_2,
            add_other_geometry_1=add_other_geometry_1,
            add_other_geometry_2=add_other_geometry_2,
            geometry_type_1=geometry_type_1,
            geometry_type_2=geometry_type_2,
            geometry_mass_1=geometry_mass_1,
            geometry_mass_2=geometry_mass_2,
            geometry_height_1=geometry_height_1,
            geometry_height_2=geometry_height_2,
            geometry_radius_1=geometry_radius_1,
            geometry_radius_2=geometry_radius_2,
            geometry_length_1=geometry_length_1,
            geometry_length_2=geometry_length_2,
            geometry_width_1=geometry_width_1,
            geometry_width_2=geometry_width_2,
            geometry_mesh_filename_1=geometry_mesh_filename_1,
            geometry_mesh_filename_2=geometry_mesh_filename_2,
            geometry_mesh_origin_xyz_1=geometry_mesh_origin_xyz_1,
            geometry_mesh_origin_xyz_2=geometry_mesh_origin_xyz_2,
            geometry_mesh_origin_rpy_1=geometry_mesh_origin_rpy_1,
            geometry_mesh_origin_rpy_2=geometry_mesh_origin_rpy_2,
            geometry_mesh_tcp_xyz_1=geometry_mesh_tcp_xyz_1,
            geometry_mesh_tcp_xyz_2=geometry_mesh_tcp_xyz_2,
            geometry_mesh_tcp_rpy_1=geometry_mesh_tcp_rpy_1,
            geometry_mesh_tcp_rpy_2=geometry_mesh_tcp_rpy_2,
        ).planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp.yaml")
    ).to_moveit_configs()

    # robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_common2.launch.py
    # ================================================================

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # ================================================================
    # from: src/manymove_planner/launch/lite_micpp_fake_action_server.launch.py
    # instead of: xarm_moveit_config/launch/_dual_robot_moveit_common2.launch.py
    # ================================================================

    # Start the actual move_group node/action server
    move_group_node_1 = Node(
        package='manymove_planner',
        executable='action_server_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'planner_type': 'moveitcpp',
                'velocity_scaling_factor': velocity_scaling_factor,
                'acceleration_scaling_factor': acceleration_scaling_factor,
                'max_exec_retries': max_exec_retries,
                'smoothing_type': smoothing_type,
                'step_size': step_size,
                'jump_threshold': jump_threshold,
                'max_cartesian_speed': max_cartesian_speed,
                'plan_number_target': plan_number_target,
                'plan_number_limit': plan_number_limit,
                'planner_prefix': prefix_1.perform(context),
                'planning_group': xarm_type_1, 
                'base_frame': base_frame.perform(context), 
                'tcp_frame': tcp_frame.perform(context), 
                'traj_controller': "{}_traj_controller".format(xarm_type_1),
            }
        ],
    )

    move_group_node_2 = Node(
        package='manymove_planner',
        executable='action_server_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'planner_type': 'moveitcpp',
                'velocity_scaling_factor': velocity_scaling_factor,
                'acceleration_scaling_factor': acceleration_scaling_factor,
                'max_exec_retries': max_exec_retries,
                'smoothing_type': smoothing_type,
                'step_size': step_size,
                'jump_threshold': jump_threshold,
                'max_cartesian_speed': max_cartesian_speed,
                'plan_number_target': plan_number_target,
                'plan_number_limit': plan_number_limit,
                'planner_prefix': prefix_2.perform(context),
                'planning_group': xarm_type_2, 
                'base_frame': base_frame.perform(context), 
                'tcp_frame': tcp_frame.perform(context), 
                'traj_controller': "{}_traj_controller".format(xarm_type_2),
            }
        ],
    )

    # Launch RViz
    rviz_config_file = (
        get_package_share_directory("manymove_planner") + "/config/micpp_demo.rviz"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_common2.launch.py
    # ================================================================

    link_base_1 = '{}link_base'.format(prefix_1.perform(context))
    link_base_2 = '{}link_base'.format(prefix_2.perform(context))

    # Static TF
    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_1.perform(context)),
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', link_base_1],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_2.perform(context)),
        output='screen',
        arguments=['0.0', '1.0', '0.0', '0.0', '0.0', '0.0', 'world', link_base_2],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    controllers = [
        '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
        '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
    ]
    if add_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_1.perform(context), robot_type_1.perform(context)))
    elif add_bio_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_1.perform(context)))
    if add_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_2.perform(context), robot_type_2.perform(context)))
    elif add_bio_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_2.perform(context)))

    # ================================================================
    # from: xarm_controller/launch/_dual_ros2_control.launch.py
    # ================================================================

    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_api'), 'launch', 'lib', 'robot_api_lib.py'))
    generate_robot_api_params = getattr(mod, 'generate_robot_api_params')
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context), node_name='ufactory_driver'
    )

    # ros2 control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_control_params,
            robot_params,
        ],
        output='screen',
    )

    # ================================================================
    # from: xarm_moveit_config/launch/_dual_robot_moveit_fake.launch.py
    # ================================================================

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    # Load controllers
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    # ================================================================
    # launch manymove_object_manager
    # ================================================================

    # Static TF
    object_manager_node = Node(
        package='manymove_object_manager',
        executable='object_manager_node',
        name='object_manager_node',
        output='screen',
        parameters=[{'frame_id': 'world'}]
    )

    # ================================================================
    # RETURN
    # ================================================================

    return [
        robot_state_publisher_node,
        move_group_node_1,
        move_group_node_2,
        rviz_node,
        static_tf_1,
        static_tf_2,
        joint_state_broadcaster,
        ros2_control_node,
        object_manager_node,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner_type', default_value='movegroup', description='Type of planner to use: moveitcpp or movegroup'),
        DeclareLaunchArgument('velocity_scaling_factor', default_value='0.5', description='Velocity scaling factor'),
        DeclareLaunchArgument('acceleration_scaling_factor', default_value='0.5', description='Acceleration scaling factor'),
        DeclareLaunchArgument('max_exec_retries', default_value='5', description='Maximum number of retries'),
        DeclareLaunchArgument('smoothing_type', default_value='time_optimal', description='Smoothing type'),
        DeclareLaunchArgument('step_size', default_value='0.05', description='Step size'),
        DeclareLaunchArgument('jump_threshold', default_value='0.0', description='Jump threshold'),
        DeclareLaunchArgument('max_cartesian_speed', default_value='0.5', description='Max cartesian speed'),
        DeclareLaunchArgument('plan_number_target', default_value='12', description='Plan number target'),
        DeclareLaunchArgument('plan_number_limit', default_value='32', description='Plan number limit'),
        DeclareLaunchArgument('base_frame', default_value='link_base', description='Base frame of the robot'),
        DeclareLaunchArgument('tcp_frame', default_value='link_tcp', description='TCP (end effector) frame of the robot' ),

        OpaqueFunction(function=launch_setup)
    ])
