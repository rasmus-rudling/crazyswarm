import sys
import os

sys.path.append(f'/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study')
sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils')

from globalVariables import NUM_TRAJECTORIES_TO_TUNE_CBF
from Participant import Participant

from GaussianProcess import GaussianProcess
from globalVariables import PATH_TO_ROOT

def stage1():
    """ Create SF3.PID """
    userInput = input("User ID or email: ")

    participantID = None

    try:
        participantID = int(userInput)
    except:
        participantEmail = userInput

    if participantID is not None:
        p = Participant.getParticipantById(participantID)
    else:
        p = Participant.getUserByEmail(participantEmail)

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
    pass


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
        