from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False}) #pornire interfata grafica

from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.motion_generation.interface_config_loader import (
    load_supported_motion_policy_config,
)

import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque

import rtde_control
import rtde_receive

#initializare world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

#deschidere fisiere pentru extragerea informatiilor
#f_real = open("/home/bogdan/Desktop/poz/poz_real.txt", "w")
#f_virt = open("/home/bogdan/Desktop/poz/poz_virt.txt", "w")

#nume robot
robot_name = "UR3e"

#calea primitivei
prim_path = "/UR3e"

#cale USD -> din //localhost -> necesita conectare la localhost
usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd"

#adaugare referinta in meniul din dreapta sus
add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

#initializare robot si reprezentare vizuala
robot_virt = world.scene.add(Robot(prim_path=prim_path, name=robot_name))

#configurare RMPFlow
rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

#initializare RMPFlow
rmpflow = RmpFlow(**rmp_config)
articulation_rmpflow = ArticulationMotionPolicy(robot_virt, rmpflow, 1.0/60)
articulation_controller = robot_virt.get_articulation_controller()

world.reset()

#initializare obiecte de control si receive
robot_rtde_control = rtde_control.RTDEControlInterface("169.254.233.2")
robot_rtde_receive = rtde_receive.RTDEReceiveInterface("169.254.233.2")

#preluare pozitii articulatii robot real
real_robot_pose = robot_rtde_receive.getActualQ()

#setare pozitii articulatii robot virtual
robot_virt.set_joint_positions(np.array(real_robot_pose))

#preluare pozitie si orientare efector
tuplu = rmpflow.get_end_effector_pose(np.array(real_robot_pose))

#pozitie
end_effec_position = tuplu[0]

#transformare orientare din matrice de rotatie in quaternioni
end_effec_rot_matr = R.from_matrix(tuplu[1])
end_effec_quat_sh = end_effec_rot_matr.as_quat()
end_effec_quat_r = deque(end_effec_quat_sh)
end_effec_quat_r.rotate(1)
end_effec_quat = list(end_effec_quat_r)

#initializare cub tinta
target_cube = cuboid.VisualCuboid(
    "/World/target", position=end_effec_position, orientation=end_effec_quat, color=np.array([0, 1, 1]), size=0.1, scale=np.array([0.1,0.1,0.1])
)

#initializare obstacol 1
band_one = cuboid.FixedCuboid(
    "/World/band_one", position=np.array([0.14, 0.379, 0]), size=0.1, scale=np.array([7, 1.5, 3.63])
)
rmpflow.add_obstacle(band_one)

#initializare obstacol 2
band_two = cuboid.VisualCuboid(
    "/World/band_two", position=np.array([0.14, -0.379, 0]), size=0.1, scale=np.array([7, 1.5, 3.63])
)
rmpflow.add_obstacle(band_two)

#poz_real = list()
#poz_virt = list()

#bucla infinita
while simulation_app.is_running():
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()
        
        #urmarire cub
        rmpflow.set_end_effector_target(target_position=target_cube.get_world_pose()[0], target_orientation=target_cube.get_world_pose()[1])

        #parametrii servoJ
        velocity = 0.1
        acceleration = 0.1
        dt = 1.0/500  # 2ms
        lookahead_time = 0.1
        gain = 300

        #preluare pozitii articulatii macheta
        joint = robot_virt.get_joint_positions()

        #transmitere spre robot real
        t_start = robot_rtde_control.initPeriod()
        robot_rtde_control.servoJ(joint, velocity, acceleration, dt, lookahead_time, gain)
        robot_rtde_control.waitPeriod(t_start)
        
        #update rmpflow si setarea noilor pozitii pentru macheta
        rmpflow.update_world()
        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)

        #corectie pozitii macheta
        robot_virt.set_joint_positions(np.array(robot_rtde_receive.getActualQ()))

        #f_real.write(str(robot_rtde_receive.getActualQ()))
        #f_virt.write(str(robot.get_joint_positions()))

    if world.is_stopped():
        #f_real.close()
        #f_virt.close()

        #deconectare interfete UR_RTDE si inchidere interfata grafica
        robot_rtde_control.servoStop()
        robot_rtde_control.stopScript()
        robot_rtde_receive.disconnect()
        simulation_app.close()

    world.step(render=True)
 # close Isaac Sim
