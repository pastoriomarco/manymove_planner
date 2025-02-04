# ========================================================================
# Contents from xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
# ========================================================================

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.30')
    report_type = LaunchConfiguration('report_type', default='dev')
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    prefix = LaunchConfiguration('prefix', default='')
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
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # ========================================================================
    # Parameters for manymove_planner package
    # ========================================================================

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

    # ========================================================================
    # Contents from xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    # ========================================================================

    ros2_control_plugin = 'uf_robot_hardware/UFRobotSystemHardware'
    controllers_name = 'controllers'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type)),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    # ========================================================================
    # WARNING: MODIFIED!!!

    # Grouped MoveItConfigsBuilder to apply the following to enable moveitcpp parameters:

    # .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    # .planning_pipelines(pipelines=["ompl"])
    # .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp_real_ufactory.yaml")

    # ========================================================================

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            controllers_name=controllers_name,
            robot_ip=robot_ip,
            report_type=report_type,
            baud_checkset=baud_checkset,
            default_gripper_baud=default_gripper_baud,
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
            ros2_control_plugin=ros2_control_plugin,
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
        .moveit_cpp(file_path=get_package_share_directory("manymove_planner") + "/config/moveit_cpp_real_ufactory.yaml")
        .to_moveit_configs()
    )
    
    # robot description launch
    # xarm_description/launch/_robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_robot_description.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
        }.items(),
    )

    # We skip moveit_config launch and we insert directly the parts we need:

    # # robot moveit common launch
    # # xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    # robot_moveit_common_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common2.launch.py'])),
    #     launch_arguments={
    #         'prefix': prefix,
    #         'attach_to': attach_to,
    #         'attach_xyz': attach_xyz,
    #         'attach_rpy': attach_rpy,
    #         'no_gui_ctrl': no_gui_ctrl,
    #         'use_sim_time': 'false',
    #         'moveit_config_dump': yaml.dump(moveit_config.to_dict()),
    #     }.items(),
    # )

    # ========================================================================
    # Contents from xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    # ========================================================================

    # We skip the original move_group_node as we start the action server that uses moveitcpp
    # The defaults on the action_server_node.cpp are made for lite6, so we don't need to input them
    # Start the actual move_group node/action server
    move_group_node = Node(
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
                'planner_prefix': prefix.perform(context),
                'planning_group': xarm_type, 
                'base_frame': base_frame.perform(context), 
                'tcp_frame': tcp_frame.perform(context), 
                'traj_controller': "{}_traj_controller".format(xarm_type),
            }
        ],
    )

    # This is from _robot_moveit_common2.launch.py, modified to load the .rviz config from manymove package

    # Launch RViz
    rviz_config_file = (
        get_package_share_directory("manymove_planner") + "/config/micpp_demo.rviz"
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config.to_dict()['robot_description'],
                'robot_description_semantic': moveit_config.to_dict()['robot_description_semantic'],
                'robot_description_kinematics': moveit_config.to_dict()['robot_description_kinematics'],
                'robot_description_planning': moveit_config.to_dict()['robot_description_planning'],
                'planning_pipelines': moveit_config.to_dict()['planning_pipelines'],
                'use_sim_time': use_sim_time
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    xyz = attach_xyz.perform(context)[1:-1].split(' ')
    rpy = attach_rpy.perform(context)[1:-1].split(' ')
    arguments = xyz + rpy + [attach_to.perform(context), '{}link_base'.format(prefix.perform(context))]

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=arguments,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # We skip the xarm_planner node as we won't use it here.
    # With this we end the contents from xarm_moveit_config/launch/_robot_moveit_common2.launch.py

    # ========================================================================
    # Contents from xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    # ========================================================================

    # joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['{}{}/joint_states'.format(prefix.perform(context), hw_ns.perform(context))]}],
        remappings=[
            ('follow_joint_trajectory', '{}{}_traj_controller/follow_joint_trajectory'.format(prefix.perform(context), xarm_type)),
        ],
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
            'ros2_control_params': ros2_control_params,
        }.items(),
    )

    control_node = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    # ================================================================
    # launch manymove_object_manager
    # ================================================================

    # Object Manager node
    manymove_object_manager_node = Node(
        package='manymove_object_manager',
        executable='object_manager_node',
        name='object_manager_node',
        output='screen',
        parameters=[{'frame_id': 'world'}]
    )

    # ================================================================
    # launch manymove_signals
    # ================================================================

    # Signals node
    manymove_signals_node = Node(
        package='manymove_signals',
        executable='signals_node',
        name='manymove_signals_node',
        output='screen',
        parameters=[{
            'robot_model': xarm_type,
            'robot_prefix': prefix
        }]
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

    return [
        robot_description_launch,
        rviz2_node,
        static_tf,
        move_group_node,
        joint_state_publisher_node,
        ros2_control_launch,
        control_node,
        manymove_object_manager_node,
        manymove_signals_node,
        manymove_hmi_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner_type', default_value='moveitcpp', description='Type of planner to use: moveitcpp or movegroup'),
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
