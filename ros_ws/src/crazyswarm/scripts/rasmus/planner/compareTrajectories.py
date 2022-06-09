import os
import sys

import numpy as np
sys.path.append("..")
from SimpleTrajectory import SimpleTrajectory
from cbFunctions import CBF
from drone import DroneGoalState, DroneState
from flyZone import MOCAP_FLY_ZONE
from motionPlanner import Planner, recordFinalTrajectory
from position import Position
from trajectory import Trajectory

def main():
  cbf = CBF(CBF.DECELERATION_MAX_MIN, CBF.EPSILON_MAX) # Slow breaking + far away from goal => conservative
  cbf = CBF(CBF.DECELERATION_MAX_MAX, CBF.EPSILON_MIN) # Hard breaking + close to the goal => liberal
  cbf = CBF(CBF.DECELERATION_MAX_MAX, CBF.EPSILON_MAX) 

  cbf = CBF(1.0, 0.1) # Fastest
  # cbf = CBF(0.1, 0.7) # Slowest

  # cbf = CBF(0.1, 0.7)

  x1, y1 = -1.5, 1.0
  x2, y2 = 1.0, -1.0

  

  planners = [
    Planner(
      dt=0.1,
      obstacles=[],
      flyZone=MOCAP_FLY_ZONE,
      verboseLevel=2,
      cbf=cbf,
      possibleAccelerations=[1.0, 0.6, 0.3, 0.0, -0.3, -0.6, -1.0]
    ),
    # Planner(
    #   dt=0.1,
    #   obstacles=[],
    #   flyZone=MOCAP_FLY_ZONE,
    #   verboseLevel=2,
    #   cbf=cbf,
    #   possibleAccelerations=[1.0, 0.5, 0.0, -0.5, -1.0]
    # ),
    Planner(
      dt=0.1,
      obstacles=[],
      flyZone=MOCAP_FLY_ZONE,
      verboseLevel=2,
      cbf=cbf,
      possibleAccelerations=[1.0, 0.3, 0.0, -0.3, -1.0]
    ),
    # Planner(
    #   dt=0.1,
    #   obstacles=[],
    #   flyZone=MOCAP_FLY_ZONE,
    #   verboseLevel=2,
    #   cbf=cbf,
    #   possibleAccelerations=[1.0, 0.0, -1.0]
    # ),
  ]

  plannerNames = []

  trajectories = []

  for i in range(len(planners)):
    # plannerName = input(f"Enter planner#{i+1} name: ")
    accs = [str(acc) for acc in planners[i].possibleAccelerations]

    plannerName = f'a=[{",".join(accs)}]'
    
    plannerNames.append(plannerName)

  comparisonIdentifier = input("Enter comparison identifier: ")
  comparisonId = f'{comparisonIdentifier} (eps={cbf.epsilon} | a_max={cbf.decceleration_max})'
  filePath = f"../savedTrajectories/comparisons/{comparisonId}"
  os.mkdir(filePath)

  for i, planner in enumerate(planners):
    goalState = DroneGoalState(x=x2, y=y2, radius=0.1)

    startingDroneState = DroneState(
      parent=None, 
      x=x1, 
      y=y1,
      yaw=planner.getYawForInitDroneState(Position(x1, y1), goalState)
    )

    plannerName = plannerNames[i]
    currentDroneState, foundGoalState = planner.findPathToGoal(startingDroneState, goalState)

    if foundGoalState:
      print("Found goal :)")
      planner.animationKey = comparisonId

      trajectory = Trajectory(finalDroneState=currentDroneState)
      trajectory.plot(f"{filePath}/plot - {plannerName}.png", cbf, planner=planner)
      trajectory.saveToCsv(f"{filePath}/trajectoryData - {plannerName}.csv")
      trajectories.append(trajectory)

      if False:
        recordFinalTrajectory(currentDroneState, planner)
      else:
        recordFinalTrajectory(currentDroneState, planner, onlyFinalFrame=True, fileName=f"trajectory - {plannerName}")

    planner.resetParams()

    if len(trajectories) == 2:
      t1 = trajectories[0]
      simpleTrajectory1 = SimpleTrajectory(timestamps=np.array(t1.timestamps), x=t1.x_positions, y=t1.y_positions, z=[-1997], yaw=t1.yaws)

      t2 = trajectories[1]
      simpleTrajectory2 = SimpleTrajectory(timestamps=np.array(t2.timestamps), x=t2.x_positions, y=t2.y_positions, z=[-1997], yaw=t2.yaws)

      simpleTrajectory1.plotTrajectory(otherTrajectory=simpleTrajectory2, fileName=f"{filePath}/plot - comparison.png")


if __name__ == "__main__":
  main()

  # - Runs -
  # Init: 5.553s
  # After CBF-update: 4.948s, 5.002s, 4.955s
  # After distance-update: 4.948s, 5.002s, 4.955s