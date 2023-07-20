import os
import numpy as np
import gym
import argparse
import configparser

from crowd_sim.envs.utils.robot import Robot

# configure environment
env_config = configparser.RawConfigParser()
env_config.read(env_config_file)
env = gym.make('CrowdSim-v0')
env.configure(env_config)
if args.square:
    env.test_sim = 'square_crossing'
if args.circle:
    env.test_sim = 'circle_crossing'
robot = Robot(env_config, 'robot')
robot.set_policy(policy)
env.set_robot(robot)
explorer = Explorer(env, robot, device, gamma=0.9)