from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time = True)


def launch_setup():
    
    ns = 'bluerov2'
               
    with sl.group(ns=ns):

        sl.node('robot_localization', 'ekf_node', name = 'ekf',
            parameters = [sl.find('ecn_auv_lab', 'ekf.yaml')],
            remappings = {'odometry/filtered': 'odom'})

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
