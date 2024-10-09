from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time=True)


def launch_setup():

    # reset simulation
    import os
    os.system("gz service -s /world/ocean/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}' >/dev/null 2>&1")

    # spawn the turbine
    sl.include('floatgen', 'farm_launch.py', launch_arguments={'gz': False, 'x': -200, 'y': -20, 'yaw': 0.})

    # spawn terrain
    with sl.group(ns = 'terrain'):
        sl.robot_state_publisher('ecn_auv_lab', 'terrain.xacro')
        sl.spawn_gz_model('terrain')

    # spawn BlueROV2
    sl.include('ecn_auv_lab', 'bluerov2_launch.py')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
