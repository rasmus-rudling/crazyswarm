from itertools import product
import sys

sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils')
from utils.globalVariables import DRONE_START_X, DRONE_START_Y, GOAL_OFFSET_X, GOAL_OFFSET_Y, POSSIBLE_ACCELERATIONS

import numpy as np
from matplotlib import pyplot as plt
from SimpleTrajectory import Z_HEIGHT, SimpleTrajectory

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
from trajectoryUtils import getLatestTrajectory; sns.set_theme()
from mpl_toolkits.mplot3d import Axes3D

import csv

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C

MAX_INPUT_VAL = 3

class GaussianProcess:
  NUM_VALUES = 50

  SF_CSV_PATH = None

  def __init__(self, currentParticipantHeight=None, safetyFunction=None):
    noise_std = 0.75
    kernel = 1 * RBF()

    self.gp = GaussianProcessRegressor(kernel=kernel, alpha=noise_std**2, n_restarts_optimizer=5)
    
    self.epsilonRange = np.linspace(CBF.EPSILON_MIN, CBF.EPSILON_MAX, self.NUM_VALUES)
    self.decelerationMaxRange = np.linspace(CBF.DECELERATION_MAX_MIN, CBF.DECELERATION_MAX_MAX, self.NUM_VALUES)

    self.predictions = np.zeros((self.epsilonRange.size, self.decelerationMaxRange.size))
    self.standard_deviations = np.zeros((self.epsilonRange.size, self.decelerationMaxRange.size))

    self.nextEpsilonToCheck = round(np.random.choice(self.epsilonRange), 2)
    self.nextdecelerationMaxToCheck = round(np.random.choice(self.decelerationMaxRange), 2)

    self.X = []
    self.safetyScores = []
    self.bestParameterPairs = []
    self.bestPerceivedSafety = []
    self.participantHeights = []

    # Check if to continue from previous data
    if safetyFunction is None:
      safetyFunction = input("Safety function name: ")

    self.sfName = safetyFunction

    if len(sys.argv) > 1 and sys.argv[1] == "plot":
      self.currentParticipantHeight = -1
    elif currentParticipantHeight is None:
      self.currentParticipantHeight = int(input("Participant height (in cm): "))
    else:
      self.currentParticipantHeight = currentParticipantHeight

    if safetyFunction is None:
      existingSafetyFunctions = [sf.split(".")[0]  for sf in os.listdir("gpData")]

      if safetyFunction in existingSafetyFunctions:
        # TODO: Lös det här för när man kommer ifrån preStudy
        GaussianProcess.SF_CSV_PATH = f"gpData/{safetyFunction}.csv"

        self.addDataFromCsv()
        self.updatePredictions()
        self.updateNextValuesToAskFor()

  def addDataFromCsv(self):
      gpData = np.loadtxt(GaussianProcess.SF_CSV_PATH, delimiter=",", skiprows=1)

      if gpData.ndim == 1:
        gpData = np.array([gpData])
      
      eps = [row[1] for row in gpData]
      a_max = [row[2] for row in gpData]

      numDataPoints = len(eps)

      self.X = [[eps[i], a_max[i]] for i in range(numDataPoints)]
      self.safetyScores = [row[3] for row in gpData]

      curr_best_eps = [row[4] for row in gpData]
      curr_best_a_max = [row[5] for row in gpData]
      self.bestParameterPairs = [[curr_best_eps[i], curr_best_a_max[i]] for i in range(numDataPoints)]
      self.bestPerceivedSafety = [row[6] for row in gpData]
      self.participantHeights = [row[7] for row in gpData]

  def addData(self, safetyScore):
    self.X.append([self.nextEpsilonToCheck, self.nextdecelerationMaxToCheck])
    self.safetyScores.append(safetyScore)

    self.updatePredictions()

    bestParameterPair = np.argmin(np.abs(self.predictions))
    eIdx, aMaxIdx = np.unravel_index(bestParameterPair, (self.NUM_VALUES, self.NUM_VALUES), order='C')

    self.bestParameterPairs.append([self.epsilonRange[eIdx], self.decelerationMaxRange[aMaxIdx]])
    self.bestPerceivedSafety.append(self.predictions[eIdx][aMaxIdx])

    self.participantHeights.append(self.currentParticipantHeight)

    self.updateCsv()

  def updateCsv(self):
    with open(f"gpData/{self.sfName}.csv", 'w', encoding='UTF8') as f:
      writer = csv.writer(f)
      header = ["idx", "eps", "a_max", "ps", "curr_best_eps", "curr_best_a_max", "curr_best_perceived_safety", "participant_height"]

      writer.writerow(header)

      numDataPoints = len(self.safetyScores)

      for i in range(numDataPoints):
        writer.writerow([ 
          i,
          self.X[i][0], 
          self.X[i][1], 
          self.safetyScores[i],
          self.bestParameterPairs[i][0],
          self.bestParameterPairs[i][1],
          self.bestPerceivedSafety[i],
          self.participantHeights[i]
        ])

  def updatePredictions(self):
    formatedSafetyScore = np.array([np.array([currentSafetyScore]) for currentSafetyScore in self.safetyScores])
    formattedX = np.array(self.X)

    self.gp.fit(formattedX, formatedSafetyScore)

    for i, eCurr in enumerate(self.epsilonRange):
      for j, aMaxCurr in enumerate(self.decelerationMaxRange):
        X_curr_to_predict = np.array([[eCurr, aMaxCurr]])
        mean_prediction, std_prediction = self.gp.predict(X_curr_to_predict, return_std=True)
        self.predictions[i, j] = mean_prediction[0]
        self.standard_deviations[i, j] = std_prediction[0]

  def updateNextValuesToAskFor(self):
    parameterComboWithLeastInfo = np.argmax(self.standard_deviations)
    eIdx, aMaxIdx = np.unravel_index(parameterComboWithLeastInfo, (self.NUM_VALUES, self.NUM_VALUES), order='C')

    self.nextEpsilonToCheck = self.epsilonRange[eIdx]
    self.nextdecelerationMaxToCheck = self.decelerationMaxRange[aMaxIdx]

  def plotCurrentPredictionAsHeatmap(self):
    ax = sns.heatmap(self.predictions)
    ax.set_ylabel("Epsilon")
    ax.set_yticklabels([str(round(e, 2)) for e in self.epsilonRange])

    ax.set_xlabel("a_max")
    ax.set_xticklabels([str(round(aMax, 2)) for aMax in self.decelerationMaxRange])

    plt.show()

  def plotCurrentPredictionAs3d(self):
    ax = plt.axes(projection='3d')

    X, Y = np.meshgrid(self.epsilonRange, self.decelerationMaxRange)

    ax.set_xlabel("Epsilon")
    ax.set_ylabel("Max deceleration")

    ax.set_zlabel("Preceived safety")
    ax.set_zlim(-MAX_INPUT_VAL, MAX_INPUT_VAL)

    ax.plot_surface(
      X, Y, self.predictions, 
      rstride=1, cstride=1, cmap='viridis', edgecolor='none'
    )

    # bestEpsIdx, bestDecMaxIdx = self.bestParameterPairs[-1]

    # N=50
    # stride=1
    # u = np.linspace(0, 2 * np.pi, N)
    # v = np.linspace(0, np.pi, N)
    # eps = (np.outer(np.cos(u), np.sin(v)) / 30) + bestEpsIdx
    # y = (np.outer(np.sin(u), np.sin(v)) / 30) + bestDecMaxIdx
    # z = (np.outer(np.ones(np.size(u)), np.cos(v))) + self.bestPerceivedSafety[-1]
    # ax.plot_surface(eps, y, z, linewidth=0.0, cstride=stride, rstride=stride)
    # ax.set_title('{0}x{0} data points, stride={1}'.format(N,stride))

    plt.draw()

    if len(sys.argv) > 1 and sys.argv[1] == "plot":
      plt.pause(9999999999999)
    plt.show()

  def getSafetyScore(self):
    def approvedRating(rating):
      return -MAX_INPUT_VAL <= rating and rating <= MAX_INPUT_VAL

    rating = -99

    while not approvedRating(rating):
      rating = input(f"\nScale: (-{MAX_INPUT_VAL} -> {MAX_INPUT_VAL} where -{MAX_INPUT_VAL} = to unsafe, 0 = perfect, {MAX_INPUT_VAL} = to safe)\nScore the prevoius two trajectories: ")
      print("")

      if rating == "q":
        exit()

      rating = int(rating)

      if not approvedRating(rating):
        print("ERROR: Rating must be in the interval (0, 10]")

    return rating

  def startProcess(self):
    print(f"\n- Round #{len(self.safetyScores)} -")
    print(f"Epsilon = {self.nextEpsilonToCheck}")
    print(f"Max deceleration = {self.nextdecelerationMaxToCheck}")
    trajectoryKeys = self.findTrajectories(saveAnimation=False)
    if not USING_MAC:
      self.executeTrajectories(trajectoryKeys)

    safetyScore = self.getSafetyScore()

    self.addData(safetyScore)
    self.updateNextValuesToAskFor()
    self.plotCurrentPredictionAs3d()

  def findTrajectories(self, saveAnimation):
    cbf = CBF(decceleration_max=self.nextdecelerationMaxToCheck, epsilon=self.nextEpsilonToCheck)
      
    planner = Planner(
      dt=0.1,
      obstacles=[],
      flyZone=MOCAP_FLY_ZONE,
      verboseLevel=3,
      cbf=cbf,
      possibleAccelerations=POSSIBLE_ACCELERATIONS
    )

    currentDroneState = Position(x=DRONE_START_X, y=DRONE_START_Y)

    goal_x, goal_y = planner.HUMAN.x + planner.HUMAN.radius + GOAL_OFFSET_X, planner.HUMAN.y + GOAL_OFFSET_Y

    goalState = DroneGoalState(x=goal_x, y=goal_y, radius=0.1)

    currentDroneState = DroneState(
      parent=None, 
      x=currentDroneState.x, 
      y=currentDroneState.y,
      yaw=planner.getYawForInitDroneState(currentDroneState, goalState)
    )

    trajectoryKeys = []

    currentDroneState, foundGoalState = planner.findPathToGoal(currentDroneState, goalState)

    if foundGoalState:
      print("Found goal :)")
      planner.setKey(currentDroneState, True)
      try:
        os.mkdir(f"savedTrajectories/{self.sfName}")
      except:
        pass
      
      planner.animationKey = f"{self.sfName}/Round #{len(self.safetyScores)} - " + planner.animationKey
      filePath = f"savedTrajectories/{planner.animationKey}"
      os.mkdir(filePath)

      trajectory = Trajectory(finalDroneState=currentDroneState)
      trajectory.plot(f"{filePath}/trajectoryPlot.png", cbf, show=False, planner=planner)
      trajectory.saveToCsv(f"{filePath}/trajectoryData.csv")

      trajectoryKeys.append(planner.animationKey)

      if saveAnimation:
        os.mkdir(f"{filePath}/animationFrames")
       
      else:
         recordFinalTrajectory(currentDroneState, planner, onlyFinalFrame=True, fileName="trajectory")

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
      currentPathToTrajectoryFolder = f"savedTrajectories/{trajectoryKey}"
      z_height = (self.currentParticipantHeight - 50) / 100
      currentPlannedTrajectory = SimpleTrajectory(csv=f"{currentPathToTrajectoryFolder}/trajectoryData.csv", z_height=z_height)
      currentStartPose = Pose(currentPlannedTrajectory.x[0], currentPlannedTrajectory.y[0], currentPlannedTrajectory.z[0], currentPlannedTrajectory.yaw[0])
      currentTrajectoryToStartPose = droneController.getTrajectoryToPose(goalPose=currentStartPose, velocity=0.5)

      pathsToTrajectories.append(currentPathToTrajectoryFolder)
      plannedTrajectories.append(currentPlannedTrajectory)
      trajectoriesToStartPoses.append(currentTrajectoryToStartPose)

    for i, trajectoryKey in enumerate(trajectoryKeys):
      trajectoryToStartPose = trajectoriesToStartPoses[i]
      plannedTrajectory = plannedTrajectories[i]
      
      if i == 0:
        recordedTrajectoryToStartPose = droneController.followTrajectory(trajectoryToStartPose)[1]
        recordedTrajectoriesToStartPoses.append(recordedTrajectoryToStartPose)

      droneController.hover(duration=2)
      recordingOfPlannedTrajectory = droneController.followTrajectory(plannedTrajectory)[1]
      recordingsOfPlannedTrajectories.append(recordingOfPlannedTrajectory)
      droneController.hover(duration=2)

    droneController.land(velocity=0.5)

    for i, trajectoryKey in enumerate(trajectoryKeys):
      plannedTrajectory = plannedTrajectories[i]
      pathToTrajectoryFolder = pathsToTrajectories[i]
      recordingOfPlannedTrajectory = recordingsOfPlannedTrajectories[i]

      plannedTrajectory.plotTrajectory(otherTrajectory=recordingOfPlannedTrajectory, fileName=f"{pathToTrajectoryFolder}/executedTrajectory")
    
   


def main():
  pt = GaussianProcess()
  
  if len(sys.argv) <= 1:
    pt.startProcess()
  elif sys.argv[1] == "plot":
    pt.plotCurrentPredictionAs3d()
   

if __name__ == "__main__":
  main()