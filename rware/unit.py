############## python utils ############3

import os
home_dir = os.environ['HOME']
python_util_dir = home_dir + "/python_utils"

import sys
sys.path.append(python_util_dir)

append_dir = python_util_dir + "/ros"
sys.path.append(append_dir)
from calculation import *

################ A start ###########

import os

from astar import *

###################################

import gym
import rware

env = gym.make("rware-tiny-1ag-v1", sensor_range=3, request_queue_size=1)

#print(env.n_agents)
#print(env.action_space[0])
#print(env.observation_space)
obs = env.reset()  # a tuple of observations
#env.agents[0].x = 1
#env.agents[0].y = 1

print(obs)

#print(actions)
#print(done)    # [False, False]
#print(reward)  # [0.0, 0.0]
#print(obs[0]["location"])

action_list = []
action_list.append((1))
#action_list.append((1,1))

# エージェントの位置を初期化

################## A Start #######################

def planning(start, goal, o):
  sx = start[0]
  sy = start[1]
  gx = goal[0]
  gy = goal[1]

  grid_size = 1.0  # [m]
  robot_radius = 0.1  # [m]

  ox, oy = [], []
  for i in range(-1, 11):
      ox.append(i)
      oy.append(-1.0)
  for i in range(-1, 11):
      ox.append(11.0)
      oy.append(i)
  for i in range(-1, 11):
      ox.append(i)
      oy.append(11.0)
  for i in range(-1, 11):
      ox.append(-1.0)
      oy.append(i)

  ox.append(11)
  oy.append(11)

  for i in range(len(o[0])):
    ox.append(o[0][i])
    oy.append(o[1][i])

  
  show_animation = False
  #if show_animation:  # pragma: no cover
  #    plt.plot(ox, oy, ".k")
  #    plt.plot(sx, sy, "og")
  #    plt.plot(gx, gy, "xb")
  #    plt.grid(True)
  #    plt.axis("equal")

  a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
  rx, ry = a_star.planning(sx, sy, gx, gy)

  rx.reverse()
  ry.reverse()

  #if show_animation:  # pragma: no cover
  #    plt.plot(rx, ry, "-r")
  #    plt.pause(0.001)
  #    plt.show()

  return rx, ry

#################################################
# **** convert a start path to action list **** #
#################################################

# dir 0: DOWN(0,-1), 1: UP(0,1), 2: LEFT(-1, 0), 3: RIGHT(1,0)

# *** action ***
#  NOOP = 0
#  FORWARD = 1
#  LEFT = 2
#  RIGHT = 3
#  TOGGLE_LOAD = 4

# s: [sx,sy,s_dir], path: [rx, ry]
def path_to_action(s, path):

  print("**** result ****")
  print(rx)
  print(ry)

  r_dir = s_dir

  drx = []
  dry = []
  for i in range(len(rx)-1):
    drx.append(rx[i+1] - rx[i])
    dry.append(ry[i+1] - ry[i])

  r_dir_vec = [0,0]

  if r_dir == 0:
    r_dir_vec = [0,-1]
  if r_dir == 1:
    r_dir_vec = [0,1]
  if r_dir == 2:
    r_dir_vec = [-1,0]
  if r_dir == 3:
    r_dir_vec = [1,0]


  action_list = []
  action_list_debug = []

  print("rotate")
  for i in range(len(rx)-1):

    target_vec = [drx[i], dry[i]]

    rotate = angle_between_vectors(r_dir_vec, target_vec)
    print(rotate)
    rotate = rotate / 90
    print(r_dir_vec, target_vec, rotate)
    r_dir_vec = target_vec
    
    if rotate > 0:
      for i in range(int(rotate)):
        # 左回転
        action_list.append((3,))
        action_list_debug.append("rotate-left")

    if rotate < 0:
      for i in range(int(-rotate)):
        # 左回転
        action_list.append((2,))
        action_list_debug.append("rotate-right")

    action_list.append((1,))
    action_list_debug.append("straight")

  return action_list
#################################################

while True:
###### path planning ######

  #### 

  sx = env.agents[0].x  # [m]
  sy = env.agents[0].y  # [m]
  gx = None  # [m]
  gy = None  # [m]

  ox = []
  oy = []
  for i in range(len(env.shelfs)):
    if env.shelfs[i] in env.request_queue:
      pick_place_x = env.shelfs[i].x
      pick_place_y = env.shelfs[i].y
    else:
      ox.append(env.shelfs[i].x)
      oy.append(env.shelfs[i].y)

      print("not req")
  
  # 荷物置き場
  ox.append(3)
  oy.append(10)

  ox.append(5)
  oy.append(10)


  rx, ry = planning([sx, sy], [pick_place_x, pick_place_y], [ox, oy])


##### convert path => action ######
  sx = env.agents[0].x  # [m]
  sy = env.agents[0].y  # [m]
  s_dir = env.agents[0].dir.value

  action_list = path_to_action([sx,sy,s_dir], [rx, ry])

  import time

  for i in range(len(action_list)):
    n_obs, reward, done, info = env.step(action_list[i])

    env.render()
    time.sleep(0.1)

# トグル
  n_obs, reward, done, info = env.step((4, ))
  env.render()
  time.sleep(1)

# 荷物置き場

  sx = env.agents[0].x  # [m]
  sy = env.agents[0].y  # [m]
  s_dir = env.agents[0].dir.value
  gx = 4  # [m]
  gy = 10  # [m]

  ox = []
  oy = []
  for i in range(len(env.shelfs)):
    ox.append(env.shelfs[i].x)
    oy.append(env.shelfs[i].y)

  # 荷物置き場 
  ox.append(3)
  oy.append(10)

  ox.append(5)
  oy.append(10)

  rx, ry = planning([sx,sy], [gx, gy], [ox, oy])

##### convert path => action ######

  action_list = path_to_action([sx,sy,s_dir], [rx, ry])

  for i in range(len(action_list)):

    n_obs, reward, done, info = env.step(action_list[i])

    env.render()
    time.sleep(0.1)

  ################ 荷物を戻す ####################
  sx = env.agents[0].x  # [m]
  sy = env.agents[0].y  # [m]
  s_dir = env.agents[0].dir.value
  gx = pick_place_x  # [m]
  gy = pick_place_y  # [m]

  ox = []
  oy = []
  for i in range(len(env.shelfs)):
    ox.append(env.shelfs[i].x)
    oy.append(env.shelfs[i].y)

  # 荷物置き場 
  ox.append(3)
  oy.append(10)

  ox.append(5)
  oy.append(10)

  rx, ry = planning([sx,sy], [gx, gy], [ox, oy])

  ##### convert path => action ######

  action_list = path_to_action([sx,sy,s_dir], [rx, ry])

  for i in range(len(action_list)):

    n_obs, reward, done, info = env.step(action_list[i])

    env.render()
    time.sleep(0.1)


# トグル
  n_obs, reward, done, info = env.step((4, ))
  env.render()
  time.sleep(1)



env.close()
