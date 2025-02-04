import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import load_python_launch_file_as_module
# from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    prefix = LaunchConfiguration('prefix', default='')

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=True)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=True)
    geometry_type = LaunchConfiguration('geometry_type', default='mesh')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.3)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='pneumatic_lite.stl')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0.03075 0 0.11885"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0.52 0"')

    # no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

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
    
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    # ros2_controllers_path
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    # from xarm_controller _ros2_control.launch.py
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_api'), 'launch', 'lib', 'robot_api_lib.py'))
    generate_robot_api_params = getattr(mod, 'generate_robot_api_params')
    robot_params = generate_robot_api_params(
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_params.yaml'),
        os.path.join(get_package_share_directory('xarm_api'), 'config', 'xarm_user_params.yaml'),
        LaunchConfiguration('ros_namespace', default='').perform(context), node_name='ufactory_driver'
    )

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            controllers_name = 'fake_controllers',
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            effort_control=effort_control,
            velocity_control=velocity_control,
            model1300=model1300,
            robot_sn=robot_sn,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            mesh_suffix=mesh_suffix,
            kinematics_suffix=kinematics_suffix,
            ros2_control_plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware',
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
            add_realsense_d435i=add_realsense_d435i,
            add_d435i_links=add_d435i_links,
            add_other_geometry=add_other_geometry,
            geometry_type=geometry_type,
            geometry_mass=geometry_mass,
            geometry_height=geometry_height,
            geometry_radius=geometry_radius,
            geometry_length=geometry_length,
            geometry_width=geometry_width,
            geometry_mesh_filename=geometry_mesh_filename,
            geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
            geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
            geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
            geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
        ).robot_description()
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp.yaml")
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()

    # Start the move_group node/action servers
    move_group_node = Node(
        package='manymove_planner',
        executable='action_server_node',
        output='screen',
        parameters=[
            moveit_config_dict,
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
                'planner_prefix': prefix.perform(context),
                'planning_group': xarm_type, 
                'base_frame': base_frame.perform(context), 
                'tcp_frame': tcp_frame.perform(context), 
                'traj_controller': "{}_traj_controller".format(xarm_type),
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

    xyz = attach_xyz.perform(context)[1:-1].split(' ')
    rpy = attach_rpy.perform(context)[1:-1].split(' ')
    arguments = xyz + rpy + [attach_to.perform(context), '{}link_base'.format(prefix.perform(context))]

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=arguments,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

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

    controllers = ['{}{}_traj_controller'.format(prefix.perform(context), xarm_type)]
    if add_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    elif add_bio_gripper.perform(context) in ('True', 'true') and robot_type.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
    )

    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager'
            ],
        ))

    # ================================================================
    # launch manymove_object_manager
    # ================================================================

    # Object Manager node
    object_manager_node = Node(
        package='manymove_object_manager',
        executable='object_manager_node',
        name='object_manager_node',
        output='screen',
        parameters=[{'frame_id': 'world'}]
    )

    # ================================================================
    # launch manymove_hmi
    # ================================================================

    # HMI node
    manymove_hmi_node = Node(
        package='manymove_hmi',
        executable='manymove_hmi_executable',
        name='manymove_hmi_node',
        output='screen',
        parameters=[{
            'robot_model': xarm_type,
            'robot_prefix': prefix
        }]
    )

    # ================================================================
    # launch manymove_cpp_trees
    # ================================================================

    # HMI node
    manymove_cpp_trees_node = Node(
        package='manymove_cpp_trees',
        executable='bt_client',
        name='manymove_cpp_tree_node',
        output='screen',
        parameters=[{
            'robot_model': xarm_type,
            'robot_prefix': prefix,
            'is_robot_real': False,
        }]
    )
    
    return [
        robot_state_publisher,
        joint_state_broadcaster,
        move_group_node,
        static_tf,
        ros2_control_node,
        rviz_node,
        object_manager_node,
        manymove_hmi_node,
        manymove_cpp_trees_node,
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