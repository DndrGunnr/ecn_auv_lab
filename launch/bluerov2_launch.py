from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('gt', True, description = 'Whether to use ground truth localization')

# initial pose
sl.declare_gazebo_axes(x=-120., y=0., yaw=0.)


def launch_setup():
    
    ns = 'bluerov2'
    
    # robot state publisher
    sl.include('bluerov2_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': ns, 'use_sim_time': sl.sim_time})
               
    with sl.group(ns=ns):
                    
        # URDF spawner to Gazebo, defaults to relative robot_description topic
        sl.spawn_gz_model(ns, spawn_args = sl.gazebo_axes_args())
            
        # ROS-Gz bridges
        bridges = []
        gz_js_topic = GazeboBridge.model_prefix(ns) + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        # pose ground truth
        bridges.append(GazeboBridge(f'/model/{ns}/pose',
                                     'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        
        # odometry
        bridges.append(GazeboBridge(f'/model/{ns}/odometry',
                                     'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros))

        # imu
        for imu in ('mpu', 'lsm'):
            bridges.append(GazeboBridge(f'{ns}/{imu}',
                          f'{imu}_raw', 'sensor_msgs/Imu', GazeboBridge.gz2ros))

        # thrusters
        for thr in range(1, 7):
            thruster = f'thruster{thr}'
            gz_thr_topic = f'/{ns}/{thruster}/cmd'
            bridges.append(GazeboBridge(gz_thr_topic, f'cmd_{thruster}', 'std_msgs/Float64', GazeboBridge.ros2gz))
        
        sl.create_gz_bridge(bridges)
                        
        # ground truth to tf if requested
        if sl.arg('gt'):
            sl.node('pose_to_tf',parameters={'child_frame': ns + '/base_link'})
        else:
            # otherwise publish ground truth as another link to get, well, ground truth
            sl.node('pose_to_tf',parameters={'child_frame': ns+'/base_link_gt'})

            # and run EKF
            sl.node('ecn_auv_lab', 'gz2ekf', parameters = sl.arg_map('use_pose'))

            sl.node('robot_localization', 'ekf_node', name = 'ekf',
                parameters = [sl.find('ecn_auv_lab', 'ekf.yaml')],
                remappings = {'odometry/filtered': 'odom'})
    
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
