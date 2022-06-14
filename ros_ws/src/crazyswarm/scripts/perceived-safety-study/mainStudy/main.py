import sys
import os


sys.path.append(f'/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study')
sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils')
sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/planner')

from cbFunctions import CBF, HeuristicSafetyFunction
from globalVariables import NUM_TRAJECTORIES_TO_TUNE_CBF
from Participant import Participant

from GaussianProcess import GaussianProcess
from globalVariables import PATH_TO_ROOT


def stage1():
    """ Create SF3.PID """
    p = Participant.getParticipant()

    try:
        os.mkdir(f"{PATH_TO_ROOT}/mainStudy/participants/{p.id}")
    except:
        pass

    try:
        os.mkdir(f"{PATH_TO_ROOT}/mainStudy/participants/{p.id}/savedTrajectories")
    except:
        pass

    gp = GaussianProcess(
        pID=p.id, 
        safetyFunction=f"sf3.{p.id}",
        csvFileName=f"participants/{p.id}/sf3.{p.id} - input.csv",
        savedTrajectoriesDir=f"{PATH_TO_ROOT}/mainStudy/participants/{p.id}/savedTrajectories"
    )
    
    for _ in range(NUM_TRAJECTORIES_TO_TUNE_CBF):
        gp.startProcess()
    

def stage2():
    """ Test safety functions """
    p = Participant.getParticipant()

    safetyFunctions = {
        "1": HeuristicSafetyFunction(),
        "2": CBF(), # TODO
        "3": CBF() # TODO
    }

    trajectories = {
        "1": None, # TODO
        "2": None, # TODO
        "3": None, # TODO
    }



def plotSafetyFunction():
    participantId = int(sys.argv[2])

    gp = GaussianProcess(
        pID=participantId, 
        safetyFunction=f"sf3.{participantId}",
        csvFileName=f"participants/{participantId}/sf3.{participantId} - input.csv",
        savedTrajectoriesDir=f"{PATH_TO_ROOT}/mainStudy/participants/{participantId}/savedTrajectories"
    )

    gp.plotCurrentPredictionAs3d()


if __name__ == "__main__":
    a = sys.argv[1]
    if len(sys.argv) > 1:
        if a == "plot":
            plotSafetyFunction()
        elif a == "s1":
            stage1()
        elif a == "s2":
            stage2()
        