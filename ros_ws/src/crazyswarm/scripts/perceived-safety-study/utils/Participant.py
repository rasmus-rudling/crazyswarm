import csv
from random import shuffle
import numpy as np
from pandas import array

from helpers import userInput
from globalVariables import PATH_TO_ROOT

# How to use:
# var conditions = ["A", "B", "C", "D"]
# balancedLatinSquare(conditions, 0)  //=> ["A", "B", "D", "C"]
# balancedLatinSquare(conditions, 1)  //=> ["B", "C", "A", "D"]
# balancedLatinSquare(conditions, 2)  //=> ["C", "D", "B", "A"]


def balancedLatinSquare(array, participantId):
    result = []
    j = 0
    h = 0

    for i in range(len(array)):
        val = 0

        if (i < 2 or i % 2 != 0):
            val = j
            j += 1
        else:
            val = len(array) - h - 1
            h += 1

        idx = (val + participantId) % len(array)
        result.append(array[idx])

    if (len(array) % 2 != 0 and participantId % 2 != 0):
        result = sorted(result, reverse=True)

    return result


PREVIOUS_DRONE_EXPERIENCE = {
    "1": "Yes, I've controlled at least one drone myself",
    "2": "Yes, I've seen at least one other person control a drone in person",
    "3":
    "No, but I've seen drones through the internet (via videos, games, etc.)",
    "4": "No",
}


class Participant:
    CSV_PATH = f"{PATH_TO_ROOT}/utils/participants.csv"

    def __init__(self, id, firstName, lastName, gender, height,
                 previousDroneExperience, email, sfOrder, evaluation):
        self.id = id
        self.firstName = firstName
        self.lastName = lastName
        self.gender = gender
        self.height = height
        self.previousDroneExperience = previousDroneExperience
        self.email = email
        self.sfOrder = sfOrder
        self.evaluation = evaluation

    def save(self):
        allParticipants = Participant.getAllParticipants()

        user = Participant.getParticipantById(id=self.id)

        updateUser = user is not None

        if updateUser:
            updatedParticipants = []

            for p in allParticipants:
                if p.id == self.id:
                    updatedParticipants.append(self)
                else:
                    updatedParticipants.append(p)

            allParticipants = updatedParticipants
        else:
            allParticipants.append(self)

        with open(Participant.CSV_PATH, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)

            header = [
                "id", "firstName", "lastName", "gender", "height",
                "previousDroneExperience", "email", "sfOrder", "evaluation"
            ]
            writer.writerow(header)

            for participant in allParticipants:
                writer.writerow([
                    participant.id, participant.firstName,
                    participant.lastName, participant.gender,
                    participant.height,
                    str(participant.previousDroneExperience),
                    participant.email, participant.sfOrder,
                    participant.evaluation
                ])

        return updateUser

    def getNextTrajectoryToEvaluate(self):
        prevEvaluations = self.evaluation.split("|")
        sfIdx = 0 if prevEvaluations[0] == "" else len(prevEvaluations)

        safetyFunctions = self.sfOrder.split("|")

        if sfIdx >= len(safetyFunctions):
            print("Nothing left to evaluate")
            return None

        return safetyFunctions[sfIdx]

    def addEvaluation(self):
        evaluation = userInput("Evaluation: ", int)

        self.evaluation += f"|{evaluation}" if self.evaluation != "" else f"{evaluation}"
        self.save()

    @staticmethod
    def getIdForNewUser():
        with open(Participant.CSV_PATH, newline='') as csvfile:
            participants = csv.reader(csvfile, delimiter=',')

            idForNewUser = len(list(participants)) - 1

            return idForNewUser

    @staticmethod
    def getAllParticipants():
        with open(Participant.CSV_PATH, newline='') as csvfile:
            participantsData = csv.reader(csvfile, delimiter=',')

            participants = []

            for i, participantData in enumerate(participantsData):
                if i != 0:
                    p = Participant(id=int(participantData[0]),
                                    firstName=participantData[1],
                                    lastName=participantData[2],
                                    gender=participantData[3],
                                    height=int(participantData[4]),
                                    previousDroneExperience=participantData[5],
                                    email=participantData[6],
                                    sfOrder=participantData[7],
                                    evaluation=participantData[8])

                    participants.append(p)

            return participants

    @staticmethod
    def getUserByEmail(email):
        allParticipants = Participant.getAllParticipants()

        for participant in allParticipants:
            if participant.email == email:
                return participant

        return None

    @staticmethod
    def getParticipantById(id):
        allParticipants = Participant.getAllParticipants()

        for participant in allParticipants:
            if participant.id == id:
                return participant

        return None

    @staticmethod
    def getParticipant():
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

        return p

    @staticmethod
    def getRandomSFOrder():
        possibleSafetyFunctions = ["1", "2", "3"]

        safetyFunctions = []

        for sfIdx in range(3):
            for _ in range(3):
                sf = possibleSafetyFunctions[sfIdx]
                safetyFunctions.append(sf)

        shuffle(safetyFunctions)

        return "|".join(safetyFunctions)

    @staticmethod
    def getLatinSquareSFOrder(pID):
        conditions = [
            "1",
            "2",
            "3",
            "1",
            "2",
            "3",
        ]
        safetyFunctions = balancedLatinSquare(conditions, pID)

        return "|".join(safetyFunctions)

    @classmethod
    def fromTerminalInput(cls: "Participant"):
        id = Participant.getIdForNewUser()
        print(f"\n- Creating new user with ID #{id} - ")
        firstName = userInput("First name: ")
        lastName = userInput("Last name: ")
        gender = userInput("Gender: ")
        height = userInput("Height (in cm): ", int)

        print("Previous drone experience?")

        for k, v in PREVIOUS_DRONE_EXPERIENCE.items():
            print(f"    {k}: {v}")

        previousDroneExperienceKey = userInput(
            "Answer (1-4): ", validAnswers={"1", "2", "3", "4"})
        previousDroneExperience = PREVIOUS_DRONE_EXPERIENCE[
            previousDroneExperienceKey]
        email = userInput("Email: ")

        newParticipant = cls(id=id,
                             firstName=firstName,
                             lastName=lastName,
                             gender=gender,
                             height=height,
                             previousDroneExperience=previousDroneExperience,
                             email=email,
                             sfOrder=Participant.getLatinSquareSFOrder(id),
                             evaluation="")

        return newParticipant

    def __str__(self):
        return f"\nID: {self.id}\nName: {self.firstName} {self.lastName}\nGender: {self.gender}\nHeight: {self.height}cm\nPrevious drone experience? {self.previousDroneExperience}\nEmail: {self.email}"


if __name__ == "__main__":
    p = Participant.fromTerminalInput()  # Create new participant
    p.save()