from itertools import product
import random
import sys

sys.path.append(
    '/home/rpl/Documents/rasmus/crazyswarm/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils'
)

from Participant import Participant
from globalVariables import DRONE_START_X, DRONE_START_Y, GOAL_OFFSET_X, GOAL_OFFSET_Y, HEIGHT_OFFSET, PATH_TO_ROOT, POSSIBLE_ACCELERATIONS
from helpers import userInput

import numpy as np
from matplotlib import pyplot as plt
from SimpleTrajectory import SimpleTrajectory

import os
import platform

USING_MAC = platform.system() == "Darwin"

if not USING_MAC:
    from crazyflieController import CrazyflieController, Pose

from planner.cbFunctions import CBF
from planner.drone import DroneGoalState, DroneState
from planner.flyZone import MOCAP_FLY_ZONE
from planner.motionPlanner import Planner, recordFinalTrajectory
from position import Position
import seaborn as sns
from trajectory import Trajectory
from trajectoryUtils import getLatestTrajectory

sns.set_theme()
from mpl_toolkits.mplot3d import Axes3D

import csv

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF

MAX_INPUT_VAL = 3

np.random.seed(201)


def getScore(gpValue):
    return abs(gpValue.safety)


class GPValue:
    def __init__(self, aMaxIdx, epsIdx, aMax, eps, safety, std):
        self.aMaxIdx = aMaxIdx
        self.epsIdx = epsIdx
        self.aMax = aMax
        self.eps = eps
        self.safety = safety
        self.std = std

    def __str__(self):
        # return f"aMaxIdx={self.aMaxIdx}\nepsIdx={self.epsIdx}\naMax={self.aMax}\neps={self.eps}\nsafety={self.safety}"
        return f"safety={self.safety}\nstd={self.std}\naMax={self.aMax}\neps={self.eps}"


class GaussianProcess:
    NUM_VALUES = 50

    def __init__(self, pID, safetyFunction, csvFileName, savedTrajectoriesDir):
        self.currentParticipant = Participant.getParticipantById(pID)
        self.sfName = safetyFunction
        self.csvFileName = csvFileName
        self.savedTrajectoriesDir = savedTrajectoriesDir

        self.initGP()
        self.initDataFromCsv()

    def initGP(self):
        noise_std = 0.75

        self.gp = GaussianProcessRegressor(kernel=RBF(),
                                           alpha=noise_std**2,
                                           n_restarts_optimizer=5)

        self.epsilonRange = np.linspace(CBF.EPSILON_MIN, CBF.EPSILON_MAX,
                                        self.NUM_VALUES)
        self.decelerationMaxRange = np.linspace(CBF.DECELERATION_MAX_MIN,
                                                CBF.DECELERATION_MAX_MAX,
                                                self.NUM_VALUES)

        self.predictions = np.zeros(
            (self.epsilonRange.size, self.decelerationMaxRange.size))
        self.standard_deviations = np.zeros(
            (self.epsilonRange.size, self.decelerationMaxRange.size))

        self.nextEpsilonToCheck = round(np.random.choice(self.epsilonRange), 2)
        self.nextdecelerationMaxToCheck = round(
            np.random.choice(self.decelerationMaxRange), 2)

    def initDataFromCsv(self):
        self.X = []
        self.perceivedSafety = []
        self.bestParameterPair = []
        self.bestPerceivedSafety = []
        self.participantID = []

        try:
            with open(self.csvFileName, newline='') as csvfile:
                csvData = list(csv.reader(csvfile, delimiter=','))[1:]

                eps = [float(row[1]) for row in csvData]
                a_max = [float(row[2]) for row in csvData]

                numDataPoints = len(eps)

                self.X = [[eps[i], a_max[i]] for i in range(numDataPoints)]
                self.perceivedSafety = [int(row[3]) for row in csvData]

                curr_best_eps = [float(row[4]) for row in csvData]
                curr_best_a_max = [float(row[5]) for row in csvData]
                self.bestParameterPair = [[
                    curr_best_eps[i], curr_best_a_max[i]
                ] for i in range(numDataPoints)]
                self.bestPerceivedSafety = [float(row[6]) for row in csvData]
                self.participantID = [int(row[7]) for row in csvData]

                self.updatePredictions()
                self.updateNextValuesToAskFor()

        except:
            print("No previous data found")

        self.updateCsv()

    @classmethod
    def throughTerminalInput(cls):
        pID = userInput("Participant id: ", int)
        safetyFunction = userInput("Safety function name: ")

        return cls(pID=pID,
                   safetyFunction=safetyFunction,
                   csvFileName=f"gpData/{safetyFunction}.csv",
                   savedTrajectoriesDir=
                   f"{PATH_TO_ROOT}/savedTrajectories/{safetyFunction}")

    def addData(self, perceivedSafety):
        self.X.append(
            [self.nextEpsilonToCheck, self.nextdecelerationMaxToCheck])
        self.perceivedSafety.append(perceivedSafety)

        self.updatePredictions()

        bestParameterPair = np.argmin(np.abs(self.predictions))
        eIdx, aMaxIdx = np.unravel_index(bestParameterPair,
                                         (self.NUM_VALUES, self.NUM_VALUES),
                                         order='C')

        self.bestParameterPair.append(
            [self.epsilonRange[eIdx], self.decelerationMaxRange[aMaxIdx]])
        self.bestPerceivedSafety.append(self.predictions[eIdx][aMaxIdx])

        self.participantID.append(self.currentParticipant.id)

        self.updateCsv()

    def updateCsv(self):
        with open(self.csvFileName, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            header = [
                "id", "eps", "aMax", "perceivedSafety", "bestEps", "bestAMax",
                "bestPerceivedSafety", "participantID"
            ]

            writer.writerow(header)

            numDataPoints = len(self.perceivedSafety)

            for i in range(numDataPoints):
                writer.writerow([
                    i, self.X[i][0], self.X[i][1], self.perceivedSafety[i],
                    self.bestParameterPair[i][0], self.bestParameterPair[i][1],
                    self.bestPerceivedSafety[i], self.participantID[i]
                ])

    def updatePredictions(self):
        formatedperceivedSafety = np.array([
            np.array([currentperceivedSafety])
            for currentperceivedSafety in self.perceivedSafety
        ])
        formattedX = np.array(self.X)

        self.gp.fit(formattedX, formatedperceivedSafety)

        for i, eCurr in enumerate(self.epsilonRange):
            for j, aMaxCurr in enumerate(self.decelerationMaxRange):
                X_curr_to_predict = np.array([[eCurr, aMaxCurr]])
                mean_prediction, std_prediction = self.gp.predict(
                    X_curr_to_predict, return_std=True)
                self.predictions[i, j] = mean_prediction[0]
                self.standard_deviations[i, j] = std_prediction[0]

    def updateNextValuesToAskFor(self):
        parameterComboWithLeastInfo = np.argmax(self.standard_deviations)
        eIdx, aMaxIdx = np.unravel_index(parameterComboWithLeastInfo,
                                         (self.NUM_VALUES, self.NUM_VALUES),
                                         order='C')

        self.nextEpsilonToCheck = self.epsilonRange[eIdx]
        self.nextdecelerationMaxToCheck = self.decelerationMaxRange[aMaxIdx]

    def plotCurrentPredictionAsHeatmap(self):
        ax = sns.heatmap(self.predictions)
        ax.set_ylabel("Epsilon")
        ax.set_yticklabels([str(round(e, 2)) for e in self.epsilonRange])

        ax.set_xlabel("a_max")
        ax.set_xticklabels(
            [str(round(aMax, 2)) for aMax in self.decelerationMaxRange])

        plt.show()

    def plotCurrentPredictionAs3d(self):
        ax = plt.axes(projection='3d')

        X, Y = np.meshgrid(self.epsilonRange, self.decelerationMaxRange)
        xx = np.vstack((X.flatten(), Y.flatten())).T

        p = self.gp.predict(xx).reshape(len(X), len(Y))

        ax.set_xlabel("Epsilon")
        ax.set_ylabel("Max deceleration")

        ax.set_zlabel("Preceived safety")
        ax.set_zlim(-MAX_INPUT_VAL, MAX_INPUT_VAL)

        ax.plot_surface(X,
                        Y,
                        p,
                        rstride=1,
                        cstride=1,
                        cmap='viridis',
                        edgecolor='none',
                        alpha=0.75,
                        zorder=100)

        ax.scatter3D(self.bestParameterPair[-1][0],
                     self.bestParameterPair[-1][1],
                     self.bestPerceivedSafety[-1],
                     color="red",
                     s=50,
                     marker="o",
                     zorder=500)

        # ------------
        gpValues = []

        for epsIdx, eps in enumerate(self.epsilonRange):
            for aMaxIdx, aMax in enumerate(self.decelerationMaxRange):
                gpValue = GPValue(
                    aMaxIdx=aMaxIdx,
                    epsIdx=epsIdx,
                    aMax=aMax,
                    eps=eps,
                    safety=self.predictions[epsIdx][aMaxIdx],
                    std=self.standard_deviations[epsIdx][aMaxIdx])

                gpValues.append(gpValue)

        numToChoose = 15

        gpValues.sort(key=getScore)

        chosenGpValues = gpValues[:numToChoose]

        for cv in chosenGpValues:
            sameEps = self.bestParameterPair[-1][0] == cv.eps
            sameAmax = self.bestParameterPair[-1][1] == cv.aMax

            if (not sameEps and not sameAmax):
                ax.scatter3D(cv.eps,
                             cv.aMax,
                             cv.safety,
                             color="blue",
                             s=50,
                             marker="o",
                             zorder=500)

        # ------------

        ax.contour(X, Y, p, [0.0])

        plt.draw()

        if len(sys.argv) > 1 and sys.argv[1] == "plot":
            plt.pause(9999999999999)

        # plt.pause(99999)
        plt.show()

    def getperceivedSafety(self):
        def approvedRating(rating):
            return -MAX_INPUT_VAL <= rating and rating <= MAX_INPUT_VAL

        rating = -99

        while not approvedRating(rating):
            rating = input(
                f"\nScale: (-{MAX_INPUT_VAL} -> {MAX_INPUT_VAL} where -{MAX_INPUT_VAL} = to unsafe, 0 = perfect, {MAX_INPUT_VAL} = to safe)\nScore the prevoius two trajectories: "
            )
            print("")

            if rating == "q":
                exit()

            rating = int(rating)

            if not approvedRating(rating):
                print(
                    f"ERROR: Rating must be in the interval [-{MAX_INPUT_VAL}, {MAX_INPUT_VAL}]"
                )

        return rating

    def startProcess(self):
        print(f"\n- Round #{len(self.perceivedSafety)} -")
        print(f"Epsilon = {self.nextEpsilonToCheck}")
        print(f"Max deceleration = {self.nextdecelerationMaxToCheck}")
        trajectoryKeys = self.findTrajectories(saveAnimation=False)
        if not USING_MAC:
            self.executeTrajectories(trajectoryKeys)

        perceivedSafety = self.getperceivedSafety()

        self.addData(perceivedSafety)
        self.updateNextValuesToAskFor()
        self.plotCurrentPredictionAs3d()

    def findTrajectories(self, saveAnimation):
        cbf = CBF(decceleration_max=self.nextdecelerationMaxToCheck,
                  epsilon=self.nextEpsilonToCheck)

        planner = Planner(dt=0.1,
                          obstacles=[],
                          flyZone=MOCAP_FLY_ZONE,
                          verboseLevel=3,
                          sf=cbf,
                          possibleAccelerations=POSSIBLE_ACCELERATIONS)

        currentDroneState = Position(x=DRONE_START_X, y=DRONE_START_Y)

        goal_x, goal_y = planner.HUMAN.x + planner.HUMAN.radius + GOAL_OFFSET_X, planner.HUMAN.y + GOAL_OFFSET_Y

        goalState = DroneGoalState(x=goal_x, y=goal_y, radius=0.1)

        currentDroneState = DroneState(parent=None,
                                       x=currentDroneState.x,
                                       y=currentDroneState.y,
                                       yaw=planner.getYawForInitDroneState(
                                           currentDroneState, goalState))

        trajectoryKeys = []

        currentDroneState, foundGoalState = planner.findPathToGoal(
            currentDroneState, goalState)

        if foundGoalState:
            print("Found goal :)")
            planner.setKey(currentDroneState, True)
            try:
                os.mkdir(f"{self.savedTrajectoriesDir}")
            except:
                pass

            dirs = self.savedTrajectoriesDir.split("/")

            if "mainStudy" in self.savedTrajectoriesDir:
                planner.animationKey = f"{dirs[-2]}/{dirs[-1]}/GP | Round #{len(self.perceivedSafety)}.{self.currentParticipant.id} - " + planner.animationKey
                filePath = f"{'/'.join(dirs[:-2])}/{planner.animationKey}"
            else:
                planner.animationKey = f"{dirs[-1]}/Round #{len(self.perceivedSafety)}.{self.currentParticipant.id} - " + planner.animationKey
                filePath = f"{'/'.join(dirs[:-1])}/{planner.animationKey}"

            os.mkdir(filePath)

            trajectory = Trajectory(finalDroneState=currentDroneState)
            trajectory.plot(f"{filePath}/trajectoryPlot.png",
                            cbf,
                            show=False,
                            planner=planner)
            trajectory.saveToCsv(f"{filePath}/trajectoryData.csv")

            trajectoryKeys.append(planner.animationKey)

            if saveAnimation:
                os.mkdir(f"{filePath}/animationFrames")

            else:
                recordFinalTrajectory(currentDroneState,
                                      planner,
                                      onlyFinalFrame=True,
                                      fileName="trajectory")

            planner.resetParams()

        return trajectoryKeys

    def executeTrajectories(self, trajectoryKeys):
        droneController = CrazyflieController()

        trajectoriesToStartPoses = []
        pathsToTrajectories = []
        plannedTrajectories = []

        recordedTrajectoriesToStartPoses = []
        recordingsOfPlannedTrajectories = []

        for i, trajectoryKey in enumerate(trajectoryKeys):
            try:
                currentPathToTrajectoryFolder = f"savedTrajectories/{trajectoryKey}"
                z_height = (self.currentParticipant.height -
                            HEIGHT_OFFSET) / 100

                currentPlannedTrajectory = SimpleTrajectory(
                    csv=f"{currentPathToTrajectoryFolder}/trajectoryData.csv",
                    z_height=z_height)
            except:
                try:
                    currentPathToTrajectoryFolder = f"{PATH_TO_ROOT}/preStudy/{trajectoryKey}"
                    z_height = (self.currentParticipant.height -
                                HEIGHT_OFFSET) / 100

                    currentPlannedTrajectory = SimpleTrajectory(
                        csv=
                        f"{currentPathToTrajectoryFolder}/trajectoryData.csv",
                        z_height=z_height)
                except:
                    currentPathToTrajectoryFolder = f"{PATH_TO_ROOT}/mainStudy/participants/{trajectoryKey}"
                    z_height = (self.currentParticipant.height -
                                HEIGHT_OFFSET) / 100

                    currentPlannedTrajectory = SimpleTrajectory(
                        csv=
                        f"{currentPathToTrajectoryFolder}/trajectoryData.csv",
                        z_height=z_height)

            currentStartPose = Pose(currentPlannedTrajectory.x[0],
                                    currentPlannedTrajectory.y[0],
                                    currentPlannedTrajectory.z[0],
                                    currentPlannedTrajectory.yaw[0])
            currentTrajectoryToStartPose = droneController.getTrajectoryToPose(
                goalPose=currentStartPose, velocity=0.5)

            pathsToTrajectories.append(currentPathToTrajectoryFolder)
            plannedTrajectories.append(currentPlannedTrajectory)
            trajectoriesToStartPoses.append(currentTrajectoryToStartPose)

        for i, trajectoryKey in enumerate(trajectoryKeys):
            trajectoryToStartPose = trajectoriesToStartPoses[i]
            plannedTrajectory = plannedTrajectories[i]

            if i == 0:
                recordedTrajectoryToStartPose = droneController.followTrajectory(
                    trajectoryToStartPose)[1]
                recordedTrajectoriesToStartPoses.append(
                    recordedTrajectoryToStartPose)

            droneController.hover(duration=2)
            recordingOfPlannedTrajectory = droneController.followTrajectory(
                plannedTrajectory)[1]
            recordingsOfPlannedTrajectories.append(
                recordingOfPlannedTrajectory)
            droneController.hover(duration=2)

        droneController.land(velocity=0.5)

        for i, trajectoryKey in enumerate(trajectoryKeys):
            plannedTrajectory = plannedTrajectories[i]
            pathToTrajectoryFolder = pathsToTrajectories[i]
            recordingOfPlannedTrajectory = recordingsOfPlannedTrajectories[i]

            plannedTrajectory.plotTrajectory(
                otherTrajectory=recordingOfPlannedTrajectory,
                fileName=f"{pathToTrajectoryFolder}/executedTrajectory")

    def setBestParameterPair(self):
        self.updatePredictions()

        gpValues = []

        for epsIdx, eps in enumerate(self.epsilonRange):
            for aMaxIdx, aMax in enumerate(self.decelerationMaxRange):
                gpValue = GPValue(
                    aMaxIdx=aMaxIdx,
                    epsIdx=epsIdx,
                    aMax=aMax,
                    eps=eps,
                    safety=self.predictions[epsIdx][aMaxIdx],
                    std=self.standard_deviations[epsIdx][aMaxIdx])

                gpValues.append(gpValue)

        numToChoose = 15

        gpValues.sort(key=getScore)

        chosenGpValues = gpValues[:numToChoose]

        chosenGPValue = np.random.choice(chosenGpValues)

        self.bestParameterPair[-1] = [chosenGPValue.eps, chosenGPValue.aMax]
        self.updateCsv()


def main():
    gp = GaussianProcess.throughTerminalInput()

    if len(sys.argv) <= 1:
        gp.startProcess()
    elif sys.argv[1] == "plot":
        gp.plotCurrentPredictionAs3d()


if __name__ == "__main__":
    main()
