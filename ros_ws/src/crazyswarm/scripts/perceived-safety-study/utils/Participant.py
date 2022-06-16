import csv
from random import shuffle
import numpy as np
from pandas import array

from helpers import userInput
from globalVariables import PATH_TO_ROOT

import itertools


class Participant:
    CSV_PATH = f"{PATH_TO_ROOT}/utils/participants.csv"

    def __init__(self, id, firstName, lastName, gender, height, email, sfOrder,
                 evaluation):
        self.id = id
        self.firstName = firstName
        self.lastName = lastName
        self.gender = gender
        self.height = height
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
                "id", "firstName", "lastName", "gender", "height", "email",
                "sfOrder", "evaluation"
            ]
            writer.writerow(header)

            for participant in allParticipants:
                writer.writerow([
                    participant.id, participant.firstName,
                    participant.lastName, participant.gender,
                    participant.height, participant.email, participant.sfOrder,
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
                                    email=participantData[5],
                                    sfOrder=participantData[6],
                                    evaluation=participantData[7])

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
    def getRandomSFOrderOld():
        possibleSafetyFunctions = ["1", "2", "3"]

        safetyFunctions = []

        for sfIdx in range(3):
            for _ in range(3):
                sf = possibleSafetyFunctions[sfIdx]
                safetyFunctions.append(sf)

        shuffle(safetyFunctions)

        return "|".join(safetyFunctions)

    @staticmethod
    def getRandomSFOrder(pID):
        possibleSafetyFunctions = ["1", "2", "3", "1", "2", "3"]

        allPermutations = list(itertools.permutations(possibleSafetyFunctions))

        np.random.seed(0)
        np.random.shuffle(allPermutations)

        firstSFCounter = {"1": 0, "2": 0, "3": 0}
        allOrders = []

        for permutation in allPermutations:
            firstSF = permutation[0]

            if firstSFCounter[firstSF] < 5:
                allOrders.append(list(permutation))
                firstSFCounter[firstSF] += 1

        participantOrder = allOrders[pID]

        return "|".join(participantOrder)

    @classmethod
    def fromTerminalInput(cls: "Participant"):
        id = Participant.getIdForNewUser()
        print(f"\n- Creating new user with ID #{id} - ")
        firstName = userInput("First name: ")
        lastName = userInput("Last name: ")
        gender = userInput("Gender: ")
        height = userInput("Height (in cm): ", int)

        email = userInput("Email: ")

        newParticipant = cls(id=id,
                             firstName=firstName,
                             lastName=lastName,
                             gender=gender,
                             height=height,
                             email=email,
                             sfOrder=Participant.getRandomSFOrder(id),
                             evaluation="")

        return newParticipant

    def __str__(self):
        return f"\nID: {self.id}\nName: {self.firstName} {self.lastName}\nGender: {self.gender}\nHeight: {self.height}cm\nEmail: {self.email}"


if __name__ == "__main__":
    p = Participant.fromTerminalInput()  # Create new participant
    p.save()
