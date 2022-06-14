import sys

sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study')
sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils')

from Participant import Participant

from GaussianProcess import GaussianProcess
from globalVariables import PATH_TO_ROOT

def updateSafetyFunction():
    userInput = input("User ID or email: ")

    participantID = None

    try:
        participantID = int(userInput)
    except:
        participantEmail = userInput

    Participant.CSV_PATH = "../utils/participants.csv"

    if participantID is not None:
        p = Participant.getParticipantById(participantID)
    else:
        p = Participant.getUserByEmail(participantEmail)

    gp = GaussianProcess(
        pID=p.id, 
        safetyFunction="sf2",
        csvFileName="sf2Input.csv",
        savedTrajectoriesDir=f"{PATH_TO_ROOT}/preStudy/savedTrajectories"
    )
    
    for _ in range(5):
        gp.startProcess()
    

if __name__ == "__main__":
    updateSafetyFunction()