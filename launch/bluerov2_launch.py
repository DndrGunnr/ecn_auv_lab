from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('gt', True, description = 'Whether to use ground truth localization')


def launch_setup():
    
    ns = 'bluerov2'
    
    # robot state publisher
    sl.include('bluerov2_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': ns, 'use_sim_time': sl.sim_time})
               
    with sl.group(ns=ns):

        # display thrusters in RViz
        sl.node('thruster_manager', 'publish_wrenches',
                parameters = {'control_frame': f'{ns}/base_link'})
                    
        # URDF spawner to Gazebo, defaults to relative robot_description topic
        sl.spawn_gz_model(ns, spawn_args = '-x -160. -y 0. -Y 0.'.split())
            
        # ROS-Gz bridges
        bridges = []
        gz_js_topic = GazeboBridge.model_prefix(ns) + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        # pose ground truth
        bridges.append(GazeboBridge(f'/model/{ns}/pose',
                                     'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        
        # imu
        for imu in ('mpu', 'lsm'):
            bridges.append(GazeboBridge(f'{ns}/{imu}',
                          f'{imu}_raw', 'sensor_msgs/Imu', GazeboBridge.gz2ros))

        # thrusters
        for thr in range(1, 7):
            thruster = f'thruster{thr}'
            gz_thr_topic = f'/{ns}/{thruster}/cmd'
            bridges.append(GazeboBridge(gz_thr_topic, f'cmd_{thruster}', 'std_msgs/Float64', GazeboBridge.ros2gz))
                        
        # ground truth to tf if requested
        if sl.arg('gt'):
            # odometry from Gz
            bridges.append(GazeboBridge(f'/model/{ns}/odometry_with_covariance',
                                     'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros, 'gz.msgs.OdometryWithCovariance'))

            sl.node('pose_to_tf',parameters={'child_frame': ns + '/base_link'})
        else:
            # otherwise publish ground truth as another link to get, well, ground truth
            sl.node('pose_to_tf',parameters={'child_frame': ns+'/base_link_gt'})
            # run sensor bridge
            sl.node('ecn_auv_lab', 'gz2ekf')

        sl.create_gz_bridge(bridges)
    
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
