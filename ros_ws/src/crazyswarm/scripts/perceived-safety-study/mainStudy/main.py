import csv
import sys
import os

sys.path.append(
    f'/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study'
)
sys.path.append(
    '/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils'
)
sys.path.append(
    '/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/planner'
)

from drone import DroneState
from motionPlanner import recordFinalTrajectory
from trajectory import Trajectory
from globalVariables import DRONE_START_X, DRONE_START_Y, GOAL_OFFSET_X, GOAL_OFFSET_Y
from drone import DroneGoalState
from motionPlanner import Planner
from cbFunctions import CBF, HeuristicSafetyFunction
from globalVariables import NUM_TRAJECTORIES_TO_TUNE_CBF
from Participant import Participant
from position import Position

from GaussianProcess import GaussianProcess
from globalVariables import PATH_TO_ROOT

x2, y2 = Planner.HUMAN.x + Planner.HUMAN.radius + GOAL_OFFSET_X, Planner.HUMAN.y + GOAL_OFFSET_Y
initDroneState = Position(x=DRONE_START_X, y=DRONE_START_Y)
goalState = DroneGoalState(x=x2, y=y2, radius=0.1)


def getSf2():
    with open(f"{PATH_TO_ROOT}/preStudy/sf2 - input.csv",
              newline='') as csvfile:
        lastRow = list(csv.reader(csvfile, delimiter=','))[-1]

        bestEps = float(lastRow[4])
        bestAMax = float(lastRow[5])

        return CBF(decceleration_max=bestAMax, epsilon=bestEps)


def getSf3(participantId):
    with open(
            f"{PATH_TO_ROOT}/mainStudy/participants/{participantId}/sf3.{participantId} - input.csv",
            newline='') as csvfile:
        lastRow = list(csv.reader(csvfile, delimiter=','))[-1]

        bestEps = float(lastRow[4])
        bestAMax = float(lastRow[5])

        return CBF(decceleration_max=bestAMax, epsilon=bestEps)


def calcSF1():
    planner = Planner(verboseLevel=3, sf=HeuristicSafetyFunction())

    currentDroneState = DroneState(parent=None,
                                   x=initDroneState.x,
                                   y=initDroneState.y,
                                   yaw=planner.getYawForInitDroneState(
                                       initDroneState, goalState))

    finalDroneState, foundGoalState = planner.findPathToGoal(
        currentDroneState, goalState)

    if foundGoalState:
        print("Found goal :)")

        planner.animationKey = f"mainStudy/SF1"
        filePath = f"{PATH_TO_ROOT}/{planner.animationKey}"

        trajectory = Trajectory(finalDroneState=finalDroneState)
        trajectory.plot(f"{filePath}/trajectoryPlot.png",
                        planner.sf,
                        planner=planner)
        trajectory.saveToCsv(f"{filePath}/trajectoryData.csv")

        try:
            os.mkdir(f"{filePath}/animationFrames")
        except:
            pass

        recordFinalTrajectory(finalDroneState, planner)


def calcSF2():
    planner = Planner(verboseLevel=3, sf=getSf2())

    currentDroneState = DroneState(parent=None,
                                   x=initDroneState.x,
                                   y=initDroneState.y,
                                   yaw=planner.getYawForInitDroneState(
                                       initDroneState, goalState))

    finalDroneState, foundGoalState = planner.findPathToGoal(
        currentDroneState, goalState)

    if foundGoalState:
        print("Found goal :)")

        planner.animationKey = f"mainStudy/SF2"
        filePath = f"{PATH_TO_ROOT}/{planner.animationKey}"

        trajectory = Trajectory(finalDroneState=finalDroneState)
        trajectory.plot(f"{filePath}/trajectoryPlot.png",
                        planner.sf,
                        planner=planner)
        trajectory.saveToCsv(f"{filePath}/trajectoryData.csv")

        try:
            os.mkdir(f"{filePath}/animationFrames")
        except:
            pass

        recordFinalTrajectory(finalDroneState, planner)


def calcSF3():
    p = Participant.getParticipant()
    planner = Planner(verboseLevel=3, sf=getSf3(p.id))

    currentDroneState = DroneState(parent=None,
                                   x=initDroneState.x,
                                   y=initDroneState.y,
                                   yaw=planner.getYawForInitDroneState(
                                       initDroneState, goalState))

    finalDroneState, foundGoalState = planner.findPathToGoal(
        currentDroneState, goalState)

    if foundGoalState:
        print("Found goal :)")

        planner.animationKey = f"{p.id}/savedTrajectories/SF3"
        filePath = f"{PATH_TO_ROOT}/mainStudy/participants/{planner.animationKey}"

        try:
            os.mkdir(filePath)
        except:
            pass

        trajectory = Trajectory(finalDroneState=finalDroneState)
        trajectory.plot(f"{filePath}/trajectoryPlot.png",
                        planner.sf,
                        planner=planner)
        trajectory.saveToCsv(f"{filePath}/trajectoryData.csv")

        recordFinalTrajectory(finalDroneState,
                              planner,
                              onlyFinalFrame=True,
                              fileName="trajectory")


def stage1():
    """ Create SF3.PID """
    p = Participant.getParticipant()

    try:
        os.mkdir(f"{PATH_TO_ROOT}/mainStudy/participants/{p.id}")
    except:
        pass

    try:
        os.mkdir(
            f"{PATH_TO_ROOT}/mainStudy/participants/{p.id}/savedTrajectories")
    except:
        pass

    gp = GaussianProcess(
        pID=p.id,
        safetyFunction=f"sf3.{p.id}",
        csvFileName=f"participants/{p.id}/sf3.{p.id} - input.csv",
        savedTrajectoriesDir=
        f"{PATH_TO_ROOT}/mainStudy/participants/{p.id}/savedTrajectories")

    gp.startProcess()


def stage2():
    """ Test safety functions """
    # TODO: implement
    pass


def plotSafetyFunction():
    participantId = int(sys.argv[2])

    gp = GaussianProcess(
        pID=participantId,
        safetyFunction=f"sf3.{participantId}",
        csvFileName=
        f"participants/{participantId}/sf3.{participantId} - input.csv",
        savedTrajectoriesDir=
        f"{PATH_TO_ROOT}/mainStudy/participants/{participantId}/savedTrajectories"
    )

    gp.plotCurrentPredictionAs3d()


if __name__ == "__main__":
    a = sys.argv[1]
    if len(sys.argv) > 1:
        if a == "plot":
            plotSafetyFunction()
        elif a == "s1":
            stage1()
        elif a == "csf1":
            calcSF1()
        elif a == "csf2":
            calcSF2()
        elif a == "csf3":
            calcSF3()
        elif a == "s2":
            stage2()