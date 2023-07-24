import os
import numpy as np
import gym
import argparse
import configparser
import matplotlib.pyplot as plt

from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.utils.action import ActionXY
from crowd_sim.envs.utils.action import ActionRot

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

plt.ion()
fig, ax = plt.subplots()

if args.visualize:
        ob = env.reset_custom(args.phase, args.test_case)
        print(f"obs:{ob}")
        done = False
        last_pos = np.array(robot.get_position())
        while not done:
            print(f"hello")
            action = ActionRot(v=0, r=0)
            ob, _, done, info, human_states = env.step_custom(action)
            current_pos = np.array(robot.get_position())
            last_pos = current_pos
            env.render(mode='live', fig=fig, ax=ax)


