import csv

vel_file = '/home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/data/rosbag2_2024_06_27-13_19_13/robot1/cmd_vel.csv'

with open(vel_file, 'r') as csvfile:
    reader = csv.reader(csvfile)
    next(reader, None)
    for row in reader:
