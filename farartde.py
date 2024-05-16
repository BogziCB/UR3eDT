from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

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

'''
import rtde_control
import rtde_receive
'''

#initializare world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

#nume robot
robot_name = "UR3e"

#prim path
prim_path = "/UR3e"

#cale USD -> din //localhost -> asigura-te ca ai localhost-ul pornit
usd_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/UniversalRobots/ur3e/ur3e.usd"

#adaugare referinta in meniul din dreapta sus
add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
rob_pos = np.array([0.25,0,0.2])
#initializare robot si reprezentare vizuala
robot = world.scene.add(Robot(prim_path=prim_path, name=robot_name))

#configurare RMPFlow
rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

#initializare RMPFlow
rmpflow = RmpFlow(**rmp_config)
articulation_rmpflow = ArticulationMotionPolicy(robot, rmpflow, 1.0/60)
articulation_controller = robot.get_articulation_controller()

obstacle = cuboid.VisualCuboid(
    "/World/obstacle", position=np.array([-0.04921, 0.45526, 0.1]), color=np.array([0, 1.0, 0]), size=0.1, scale=np.array([4, 4, 5])
)
rmpflow.add_obstacle(obstacle)

world.reset()
#rtde_receive

'''
robot_rtde_control = rtde_control.RTDEControlInterface("169.254.233.2")
robot_rtde_receive = rtde_receive.RTDEReceiveInterface("169.254.233.2")
real_robot_pose = robot_rtde_receive.getActualQ()
robot.set_joint_positions(np.array(real_robot_pose))

tuplu = rmpflow.get_end_effector_pose(np.array(real_robot_pose))
end_effec_position = tuplu[0]
end_effec_rot_matr = R.from_matrix(tuplu[1])
end_effec_quat_sh = end_effec_rot_matr.as_quat()

end_effec_quat_r = deque(end_effec_quat_sh)
end_effec_quat_r.rotate(1)
end_effec_quat = list(end_effec_quat_r)
'''
target_cube = cuboid.VisualCuboid(
    "/World/target", position=[0.5,0.5,0.2], orientation=[0,0,1,0], color=np.array([0, 1, 1]), size=0.1, scale=np.array([0.1,0.1,0.1])
)

while simulation_app.is_running():
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()
        
        
        #urmarire cub
        rmpflow.set_end_effector_target(target_position=target_cube.get_world_pose()[0], target_orientation=target_cube.get_world_pose()[1])
        '''
        velocity = 0.1
        acceleration = 0.1
        dt = 1.0/500  # 2ms
        lookahead_time = 0.1
        gain = 300

        joint = robot.get_joint_positions()

        t_start = robot_rtde_control.initPeriod()
        robot_rtde_control.servoJ(joint, velocity, acceleration, dt, lookahead_time, gain)

        robot_rtde_control.waitPeriod(t_start)
        '''
        rmpflow.update_world()
        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)
        
    if world.is_stopped():
        '''
        robot_rtde_control.servoStop()
        robot_rtde_control.stopScript()
        robot_rtde_receive.disconnect()
        '''
        simulation_app.close()

    world.step(render=True)
 # close Isaac Sim
