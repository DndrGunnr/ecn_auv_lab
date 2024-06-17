from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time=True)


def launch_setup():

    # run the simulation
    sl.include('floatgen', 'farm_launch.py',
               launch_arguments={'x': -200, 'y': -20, 'yaw': 3.4})

    # spawn terrain
    with sl.group(ns = 'terrain'):
        sl.robot_state_publisher('ecn_auv_lab', 'terrain.xacro',
                                 remappings = {'robot_description': 'description'})
        sl.spawn_gz_model('terrain', topic = 'description')

    # display in RViz
    sl.rviz(sl.find('ecn_auv_lab', 'layout.rviz'))

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
