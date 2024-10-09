from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('manual', default_value=True)


def launch_setup():

    with sl.group(ns='bluerov2'):

        # load body controller
        sl.node('auv_control', 'cascaded_pid', parameters=[sl.find('ecn_auv_lab', 'cascaded_pid.yaml')],
                output='screen')

        if sl.arg('manual'):
            sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                    arguments=[sl.find('ecn_auv_lab', 'pose_setpoint.yaml')])
        else:
            # uncomment your node here to run it easily when it works
            #sl.node('ecn_auv_lab', 'waypoints')
            pass


    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
