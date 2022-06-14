import os
from planner.cbFunctions import CBF, HeuristicSafetyFunction
from planner.drone import DroneGoalState, DroneState
from planner.flyZone import MOCAP_FLY_ZONE
from planner.motionPlanner import Planner, recordFinalTrajectory
from position import Position
from trajectory import Trajectory
import cProfile, pstats, io
from pstats import SortKey
from time import time

from utils.globalVariables import DRONE_START_X, DRONE_START_Y, GOAL_OFFSET_X, GOAL_OFFSET_Y


def main():
    sf = CBF(
        CBF.DECELERATION_MAX_MIN,
        CBF.EPSILON_MAX)  # Slow breaking + far away from goal => conservative

    sf = CBF(CBF.DECELERATION_MAX_MAX,
             CBF.EPSILON_MIN)  # Hard breaking + close to the goal => liberal
    sf = CBF(CBF.DECELERATION_MAX_MAX, CBF.EPSILON_MAX)

    # sf = CBF(1.0, 0.1)  # Fastest
    # sf = CBF(0.1, 0.7)  # Slowest

    planner = Planner(
        verboseLevel=3,
        sf=sf,
    )

    x2, y2 = planner.HUMAN.x + planner.HUMAN.radius + GOAL_OFFSET_X, planner.HUMAN.y + GOAL_OFFSET_Y

    currentDroneState = Position(x=DRONE_START_X, y=DRONE_START_Y)

    goalState = DroneGoalState(x=x2, y=y2, radius=0.1)

    currentDroneState = DroneState(parent=None,
                                   x=currentDroneState.x,
                                   y=currentDroneState.y,
                                   yaw=planner.getYawForInitDroneState(
                                       currentDroneState, goalState))

    currentDroneState, foundGoalState = planner.findPathToGoal(
        currentDroneState, goalState)

    if foundGoalState:
        print("Found goal :)")
        planner.setKey(currentDroneState, True)
        filePath = f"savedTrajectories/{planner.animationKey}"
        os.mkdir(filePath)

        trajectory = Trajectory(finalDroneState=currentDroneState)
        trajectory.plot(f"{filePath}/trajectoryPlot.png", sf, planner=planner)
        trajectory.saveToCsv(f"{filePath}/trajectoryData.csv")

        os.mkdir(f"{filePath}/animationFrames")

        if False:
            recordFinalTrajectory(currentDroneState, planner)
        else:
            recordFinalTrajectory(currentDroneState,
                                  planner,
                                  onlyFinalFrame=True)

    planner.resetParams()


if __name__ == "__main__":
    main()

    # - Runs -
    # Init: 5.553s
    # After CBF-update: 4.948s, 5.002s, 4.955s
    # After distance-update: 4.948s, 5.002s, 4.955s
