import sys

sys.path.append(
    '/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study'
)
sys.path.append(
    '/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils'
)
from globalVariables import NUM_TRAJECTORIES_TO_TUNE_CBF

from Participant import Participant

from GaussianProcess import GaussianProcess
from globalVariables import PATH_TO_ROOT


def updateSafetyFunction():
    gp = GaussianProcess(
        pID=int(sys.argv[1]),
        safetyFunction="sf2",
        csvFileName="sf2 - input.csv",
        savedTrajectoriesDir=f"{PATH_TO_ROOT}/preStudy/savedTrajectories")

    gp.startProcess()


def plotSafetyFunction():
    gp = GaussianProcess(
        pID=0,
        safetyFunction="sf2",
        csvFileName="sf2 - input.csv",
        savedTrajectoriesDir=f"{PATH_TO_ROOT}/preStudy/savedTrajectories")

    gp.plotCurrentPredictionAs3d()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "plot":
        plotSafetyFunction()
    else:
        updateSafetyFunction()