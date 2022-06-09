#!/usr/bin/env python
import csv
import sys
import matplotlib.pyplot as plt
from tracemalloc import start
import time
from TrajectoryUtils import TrajectoryUtils
from TrajectoryUtils import euclidianDistance
sys.path.append("../..")

from planner.cbFunctions import heuristicFunction

from pycrazyswarm import Crazyswarm
import numpy as np

MOCAP_ROOM_HEIGHT = 2.57
MOCAP_ROOM_LENGTH = 4.5
MOCAP_ROOM_WIDTH = 3.55

HUMAN_RADIUS = 0.5
DRONE_RADIUS = 0.2
MIN_ALLOWED_DISTANCE_TO_HUMAN = 0.45

def getMaxVelocity(dronePos):
  x, y, _ = dronePos
  distanceToHuman = euclidianDistance(x, y, 0, 0)
  maxAllowedVelocity = heuristicFunction(distanceToHuman)

  return maxAllowedVelocity

def getSleepTime(velocity, tentacleLength):
  return tentacleLength / velocity


def executeCustomTrajectory(timeHelper, drone, rate=100, trajectoryLogger=None, trajectoryToFollow=None, droneHeight=2):
    num_timestamps = len(trajectoryToFollow.timestamps)

    for event_idx in range(num_timestamps):
        currentPosition = trajectoryToFollow.positions[event_idx]
        currentPosition[2] = droneHeight

        currentVelocity = trajectoryToFollow.velocities[event_idx]
        currentAcceleration = trajectoryToFollow.accelerations[event_idx]
        currentYaw = trajectoryToFollow.yaws[event_idx]
        currentOmega = trajectoryToFollow.omegas[event_idx]

        currentTimestamp = trajectoryToFollow.timestamps[event_idx]

        drone.cmdFullState(
          pos=currentPosition,
          vel=currentVelocity,
          acc=currentAcceleration,
          yaw=currentYaw % (2 * np.pi),
          omega=currentOmega
        )

        # timeHelper.sleepForRate(rate)
        v = getMaxVelocity(currentPosition)
        sleepTime = getSleepTime(v, trajectoryToFollow.tentacleLength)

        timeHelper.sleep(sleepTime)
        trajectoryLogger.appendDroneEvent(currentTimestamp, drone)
        

if __name__ == "__main__":
    trajectoryToFollow = TrajectoryUtils("csvs/PathPlanningTrajectory.csv")

    startX = trajectoryToFollow.positions[0][0]
    startY = trajectoryToFollow.positions[0][1]
    startYaw = trajectoryToFollow.yaws[0]

    obstacleRadius = obstacleToPlot.radius - DRONE_RADIUS

    crazyflies_yaml = str({'crazyflies': [{'channel': 100, 'id': 7, 'initialPosition': [startX, startY, startYaw], 'type': 'default'}]})
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml, obstacleRadius=obstacleRadius)
    timeHelper = swarm.timeHelper
    drone = swarm.allcfs.crazyflies[0]

    plt.gca().view_init(elev=-90, azim=-90)

    droneHeight = 2

    drone.takeoff(targetHeight=droneHeight, duration=2)
    timeHelper.sleep(2)

    rate = 15 # In Hz
    trajectoryLogger = TrajectoryUtils()

    executeCustomTrajectory(timeHelper, drone, rate, trajectoryLogger, trajectoryToFollow, droneHeight)

    trajectoryLogger.saveTrajectoryToCsv('csvs/loggedLatticeTrajectory.csv')
  # trajectoryLogger.compareWithOtherTrajectory(trajectoryToFollow)

    print("Follower done!")

    drone.notifySetpointsStop()
    drone.land(targetHeight=0.03, duration=0.5)
    timeHelper.sleep(0.5)
    plt.close()

    
