import os
from matplotlib import pyplot as plt
import numpy as np
import math
import pandas as pd

import sys
sys.path.append("..")
from TrajectoryUtils import *

try:
    import model_predictive_trajectory_generator as planner
    import motion_model
except ImportError:
    raise

# show_animation = True
show_animation = False


def search_nearest_one_from_lookuptable(tx, ty, tyaw, lookup_table):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookup_table):
        dx = tx - table[0]
        dy = ty - table[1]
        dyaw = tyaw - table[2]
        d = math.sqrt(dx ** 2 + dy ** 2 + dyaw ** 2)
        if d <= mind:
            minid = i
            mind = d

    return lookup_table[minid]


def get_lookup_table():
    table_path = os.path.dirname(os.path.abspath(__file__)) + "/lookuptable.csv"
    data = pd.read_csv(table_path)

    return np.array(data)


def sample_states(angle_samples, a_min, a_max, d, p_max, p_min, nh):
    states = []
    for i in angle_samples:
        a = a_min + (a_max - a_min) * i

        for j in range(nh):
            xf = d * math.cos(a)
            yf = d * math.sin(a)
            if nh == 1:
                yawf = (p_max - p_min) / 2 + a
            else:
                yawf = p_min + (p_max - p_min) * j / (nh - 1) + a
            states.append([xf, yf, yawf])

    return states


def calc_uniform_polar_states(nxy, nh, d, a_min, a_max, p_min, p_max):
    """
    calc uniform state

    :param nxy: number of position sampling
    :param nh: number of heading sampleing
    :param d: distance of terminal state
    :param a_min: position sampling min angle
    :param a_max: position sampling max angle
    :param p_min: heading sampling min angle
    :param p_max: heading sampling max angle
    :return: states list
    """

    angle_samples = [i / (nxy - 1) for i in range(nxy)]
    states = sample_states(angle_samples, a_min, a_max, d, p_max, p_min, nh)

    return states


def generate_path(target_states, k0):
    # x, y, yaw, s, km, kf
    lookup_table = get_lookup_table()
    result = []

    for state in target_states:
        bestp = search_nearest_one_from_lookuptable(
            state[0], state[1], state[2], lookup_table)

        target = motion_model.State(x=state[0], y=state[1], yaw=state[2])
        init_p = np.array(
            [math.sqrt(state[0] ** 2 + state[1] ** 2), bestp[4], bestp[5]]).reshape(3, 1)

        x, y, yaw, p, dt = planner.optimize_trajectory(target, k0, init_p)

        if x is not None:
            # print("find good path")
            result.append(
                [x[-1], y[-1], yaw[-1], float(p[0]), float(p[1]), float(p[2])])

            for res in result:
                # x_print, y_print, yaw_print = result[-1][0], result[-1][1], result[-1][2]
                x_print, y_print, yaw_print = res[0], res[1], res[2]
                # print(
                #     f"[x, y, yaw]=[{x_print}, {y_print}, {np.rad2deg(yaw_print)}°]")
                # exit(0)

    print("finish path generation")
    return result


def uniform_terminal_state_sampling(tentacleLength):
    k0 = 0.0
    nxy = 3
    nh = 3
    d = tentacleLength
    MAX_DEGREE = 5
    a_min = - np.deg2rad(MAX_DEGREE)
    a_max = np.deg2rad(MAX_DEGREE)
    p_min = - np.deg2rad(MAX_DEGREE)
    p_max = np.deg2rad(MAX_DEGREE)
    states = calc_uniform_polar_states(nxy, nh, d, a_min, a_max, p_min, p_max)
    result = generate_path(states, k0)

    resX, resY, resYaw, dt = [], [], [], None

    for table in result:
        xc, yc, yawc, dt = motion_model.generate_trajectory(
            table[3], table[4], table[5], k0)

        resX.append(xc)
        resY.append(yc)
        resYaw.append(yawc)

        if show_animation:
            plt.plot(xc, yc, "-r")

    if show_animation:
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    return resX, resY, resYaw, dt

def getLatticePaths(tentacleLength=10):
    planner.show_animation = show_animation
    resX, resY, resYaw, dt = uniform_terminal_state_sampling(tentacleLength)

    straightIdx = 6

    latticeIndices = [2, straightIdx, -3]
    
    directions = {
      2: "right",
      straightIdx: "straight",
      -3: "left"
    }

    latticePaths = []

    tentacleDivider = 100

    realTentacleLength = tentacleLength / tentacleDivider

    for latticeIdx in latticeIndices:
      if latticeIdx == straightIdx:
        numPoints = len(resX[0])
        currentLatticeX = np.linspace(0, realTentacleLength, numPoints)
        currentLatticeY = np.zeros(numPoints)
      else:
        currentLatticeX = np.array(resX[latticeIdx]) / tentacleDivider
        currentLatticeY = np.array(resY[latticeIdx]) / tentacleDivider
        currentLatticeYaw = resYaw[latticeIdx]

      direction = directions[latticeIdx]

      currentLatticePath = LatticePath(currentLatticeX, currentLatticeY, direction)
      latticePaths.append(currentLatticePath)

    return latticePaths, realTentacleLength

def main():
    latticePaths = getLatticePaths()[0]

    trajectoryPlanner = TrajectoryPlanner(
      latticePaths=latticePaths,
      obstacle=Obstacle(0, 0, 2),
      dt=0.1, 
      startYaw=0, 
      startX=1.5, 
      startY=1.5, 
      goalX=-2.1, 
      goalY=-1.5
    )

    trajectoryToGoal = trajectoryPlanner.findPathToGoal()


if __name__ == "__main__":
  main()