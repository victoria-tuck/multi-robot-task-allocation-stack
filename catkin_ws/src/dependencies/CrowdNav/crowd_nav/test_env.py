import os
import numpy as np
import gym
import argparse
import configparser
import matplotlib.pyplot as plt

from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.utils.action import ActionXY
from crowd_sim.envs.utils.action import ActionRot

from cbf_controller_feasible_barrier import cbf_controller

parser = argparse.ArgumentParser('Parse configuration file')
parser.add_argument('--env_config', type=str, default='crowd_nav/configs/env.config')
parser.add_argument('--policy_config', type=str, default='crowd_nav/configs/policy.config')
parser.add_argument('--visualize', default=True, action='store_true')
parser.add_argument('--phase', type=str, default='test')
parser.add_argument('--test_case', type=int, default=None)
parser.add_argument('--square', default=False, action='store_true')
parser.add_argument('--circle', default=False, action='store_true')
parser.add_argument('--video_file', type=str, default=None)
parser.add_argument('--traj', default=False, action='store_true')
args = parser.parse_args()

# configure environment
env_config = configparser.RawConfigParser()
env_config.read(args.env_config)
env = gym.make('CrowdSim-v0')
env.configure(env_config)
if args.square:
    env.test_sim = 'square_crossing'
if args.circle:
    env.test_sim = 'circle_crossing'
robot = Robot(env_config, 'robot')
env.set_robot(robot)
robot.print_info()

# controller = cbf_controller(  )

plt.ion()
fig, ax = plt.subplots()

if args.visualize:
        ob = env.reset_custom(args.phase, args.test_case)
        print(f"obs:{ob}")
        done = False
        last_pos = np.array(robot.get_position())

        human_states, human_states_dot = env.get_human_states()
        while not done:

            # robot_full_state = robot.get_full_state()
            # robot_state = np.array([ robot_full_state.px, robot_full_state.py, robot_full_state.theta, robot_full_state.vx*np.cos(robot_full_state.theta)+robot_full_state.vy*np.sin(robot_full_state.theta) ]).reshape(-1,1)
            # robot_goal = np.array([robot_full_state.gx, robot_full_state.gy]).reshape(-1,1)
            # robot_radius = robot_full_state.radius
            # control_input = controller.policy( robot_state, robot_goal, robot_radius, human_states, human_states_dot )
            # action = ActionRot(v=control_input[0,0], r=control_input[1,0])
            action = ActionRot(v=0, r=0)
            ob, _, done, info, human_states, human_states_dot = env.step_custom(action)
            print(f"robotX: {robot_full_state}")
            current_pos = np.array(robot.get_position())
            # print(f"  ")
            last_pos = current_pos
            env.render(mode='live', fig=fig, ax=ax)


