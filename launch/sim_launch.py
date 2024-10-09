from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time=True)


def launch_setup():

    # run the simulation
    sl.gz_launch(sl.find('ecn_auv_lab', 'world.sdf'), '-r')

    # display in RViz
    sl.rviz(sl.find('ecn_auv_lab', 'layout.rviz'))

    # spawn stuff
    sl.include('ecn_auv_lab', 'spawn_all_launch.py')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
