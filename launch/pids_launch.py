from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('sliders', default_value=True)
    sl.declare_arg('rviz', default_value=True)
    
    with sl.group(ns=sl.arg('namespace')):
        
        # load body controller anyway
        sl.node('auv_control', 'cascaded_pid', parameters=[sl.find('ecn_auv_lab', 'cascaded_pid.yaml')],
                output='screen')

        with sl.group(if_arg='sliders'):
            sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                    arguments=[sl.find('ecn_auv_lab', 'pose_setpoint.yaml')])

    return sl.launch_description()
