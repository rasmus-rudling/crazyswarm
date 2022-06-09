from audioop import reverse
from datetime import datetime
import math
from time import time
from typing import List, Tuple
from matplotlib.axes import Axes
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np

from planner.cbFunctions import CBF, heuristicFunction, isApprovedByCbf
from planner.drone import DroneGoalState, DroneState
from planner.flyZone import FlyZone
from planner.helperFunctions import reverse_insort
from planner.movement import Movement

from planner.obstacle import Obstacle
from planner.plotting import plotPathToCurrentDroneStateLive, plotlive
from position import Position
import os

from threading import Thread

import glob
from PIL import Image

from trajectory import Trajectory

PRINT_AT = 5000

class Planner:
  CONTROLL_BARRIER_FUNCTIONS = {
    "heuristic": heuristicFunction,
    "cbf": isApprovedByCbf
  }

  HUMAN = Obstacle(0, 0, 0.25) # If you change here, change in plotting aswell

  def __init__(self, dt: float, obstacles: List["Obstacle"], flyZone: "FlyZone", verboseLevel: int, cbf: "CBF", possibleAccelerations=[1.0, 0.6, 0.3, 0.0, -0.3, -0.6, -1.0]) -> None:
    self.dt = dt
    # Movement.POSSIBLE_ACCELERATIONS = [acc * dt for acc in possibleAccelerations]
    self.originalPossibleAccelerations = possibleAccelerations
    self.possibleAccelerations = [acc * dt for acc in possibleAccelerations]
    self.obstacles = obstacles
    self.flyZone = flyZone
    self.verboseLevel = verboseLevel
    self.cbf = cbf

    self.iterations = 0
    self.currentGoalState = None
    self.currentStartState = None
    self.numIterationsForLastPath = None
    self.movements = []
    self.visitedMovements = set()

     # Animation
    self.saveAnimation = False
    self.lengthOfAnimation = 10
    self.animationFrameDuration = 0.25  # In seconds
    self.numAnimationFrames = self.lengthOfAnimation / self.animationFrameDuration
    self.currentAnimationFrame = 0
    self.animationKey = None

    self.computationTime = -1

  @plotlive
  def plotStateLive(self, droneState: "DroneState", ax: Axes, fig: Figure, isFinalDroneState: bool, fileName=None) -> None:
    plotPathToCurrentDroneStateLive(
      droneState=droneState, 
      planner=self,
      ax=ax,
      fig=fig,
      isFinalDroneState=isFinalDroneState,
      fileName=fileName
    )
    # time.sleep(0.1)


  def printMovements(self) -> None:
    print("\n-Movements-")
    for movement in self.movements:
      print(movement)
    print("---")


  def animationPlot(self, droneState: "DroneState", ax: Axes, fig: Figure, isFinalDroneState:bool=False) -> None:
    if self.numIterationsForLastPath is not None:
      divider = self.numAnimationFrames - 3
      plotAt = self.numIterationsForLastPath // divider
    
      if plotAt == 0.0:
        plotAt = 1
      
      plotNow = self.iterations % plotAt == 0 or self.iterations == 0

      if plotNow or isFinalDroneState:
        self.currentAnimationFrame += 1
        self.plotStateLive(droneState, ax, fig, isFinalDroneState)


  def printStatus(self, droneState: "DroneState") -> None:
    print()
    print(f"#{self.iterations}\t\tC={round(droneState.selectedMovement.newDistanceToGoal, 3)}m\t\tv={round(droneState.velocity, 3)}m/s")


  def getYawForInitDroneState(self, initDronePos: "Position", goalState: "DroneGoalState"):
    initYaw = math.atan2(goalState.y - initDronePos.y, goalState.x - initDronePos.x)
    return initYaw


  def getFinalDroneState(self, movementThatReachedGoal: "Movement"):
    finalDroneState = movementThatReachedGoal.associatedDroneState
    finalDroneState.selectedMovement = movementThatReachedGoal

    # finalDroneState = DroneState(penultimateDroneState)
    return finalDroneState

  def addMovements(self, associatedDroneState: "DroneState") -> Tuple[bool, "Movement"]:
    angles = [
      np.deg2rad(-5.0), 
      np.deg2rad(0.0), 
      np.deg2rad(5.0)
    ]

    accelerations = self.possibleAccelerations

    closeDistance = 0.5

    if associatedDroneState.parent is not None and associatedDroneState.parent.selectedMovement.newDistanceToGoal < closeDistance**2:
      angles = [np.deg2rad(0.0)] # Only straight

      accelerations = []

      for acc in self.possibleAccelerations:
        newVelocity = associatedDroneState.parent.selectedMovement.velocity + acc

        if newVelocity <= 0.75:
          accelerations.append(acc)

    for angle in angles:
      for acceleration in accelerations:
        newMovement = Movement(
          angle=angle,
          acceleration=acceleration,
          planner=self,
          associatedDroneState=associatedDroneState
        )

        if newMovement.willHaveReachedGoal:
          return True, newMovement
        elif newMovement.isValid and newMovement.key not in self.visitedMovements:
          reverse_insort(self.movements, newMovement)
          self.visitedMovements.add(newMovement.key)
    
    numMovements = len(self.movements)
    # valuesToKeep = 5000
    # startIdx = max(numMovements-valuesToKeep, 0)
    # self.movements = self.movements[startIdx:numMovements]

    if (self.verboseLevel == 2 or self.verboseLevel == 3) and self.iterations % PRINT_AT == 0:
      print(f"#movements = {numMovements}")

    return False, None

  def addMovementsWithThreading(self, associatedDroneState: "DroneState", multiThread=False) -> Tuple[bool, "Movement"]:
    def addNewMovement(angle, acceleration, info):
      newMovement = Movement(
          angle=angle,
          acceleration=acceleration,
          planner=self,
          associatedDroneState=associatedDroneState
        )

      if newMovement.willHaveReachedGoal:
        info[0] = True
      elif newMovement.isValid and newMovement.key not in self.visitedMovements:
        reverse_insort(self.movements, newMovement)
        self.visitedMovements.add(newMovement.key)

      info[1] = newMovement

    
    if multiThread:
      threadTasks = []
      infoPerThread = [[False, None]] * (len(Movement.POSSIBLE_ANGLES) * len(self.possibleAccelerations))

      threadIdx = 0

      for angle in Movement.POSSIBLE_ANGLES:
        for acceleration in self.possibleAccelerations:
          t = Thread(target=addNewMovement, args=(angle, acceleration, infoPerThread[threadIdx]))
          t.start()
          threadTasks.append(t)

          threadIdx += 1

      for tIdx, threadTask in enumerate(threadTasks):
        threadTask.join()
        reachedGoal = infoPerThread[tIdx][0]

        if reachedGoal:
            return infoPerThread[tIdx]

    else:
      info = [False, None]

      for angle in Movement.POSSIBLE_ANGLES:
        for acceleration in self.possibleAccelerations:
          addNewMovement(angle, acceleration, info)

          reachedGoal = info[0]

          if reachedGoal:
            return info

    numMovements = len(self.movements)
    valuesToKeep = 5000
    startIdx = max(numMovements-valuesToKeep, 0)
    self.movements = self.movements[startIdx:numMovements]

    if (self.verboseLevel == 2 or self.verboseLevel == 3) and self.iterations % PRINT_AT == 0:
      print(f"#movements = {numMovements}")

    return False, None


  def findPathToGoal(self, initDroneState: "DroneState", goalState: "DroneGoalState"):
    startTime = time()
    
    self.currentGoalState = goalState
    self.currentStartState = initDroneState

    reachedGoal, movementThatReachedGoal = self.addMovements(initDroneState)

    if (self.saveAnimation and self.numIterationsForLastPath is not None) or self.verboseLevel == 1 or self.verboseLevel == 3:
      fig, ax = plt.subplots()
    else:
      fig, ax = None, None


    if self.saveAnimation:
      self.animationPlot(initDroneState, ax, fig)

    if reachedGoal:
      finalDroneState = self.getFinalDroneState(movementThatReachedGoal)
      return finalDroneState, True

    while len(self.movements) > 0:
      self.iterations += 1

      cheapestMovement = self.movements.pop()

      associatedDroneState = cheapestMovement.associatedDroneState

      associatedDroneState.selectedMovement = cheapestMovement

      newDroneState = DroneState(associatedDroneState)

      if self.verboseLevel == 1 and self.iterations % PRINT_AT == 0:
        self.plotStateLive(newDroneState, ax, fig, False)
      elif self.verboseLevel == 2 and self.iterations % PRINT_AT == 0:
        self.printStatus(associatedDroneState)
      elif self.verboseLevel == 3 and self.iterations % PRINT_AT == 0:
        self.plotStateLive(newDroneState, ax, fig, False)
        self.printStatus(associatedDroneState)
      

      if self.saveAnimation:
        self.animationPlot(newDroneState, ax, fig)

      reachedGoal, movementThatReachedGoal = self.addMovements(newDroneState)

      if reachedGoal:
        finalDroneState = self.getFinalDroneState(movementThatReachedGoal)
          
        if self.saveAnimation:
          self.animationPlot(finalDroneState, ax, fig, True)

        endTime = time()
        self.computationTime = endTime - startTime

        return finalDroneState, True

    endTime = time()
    self.computationTime = endTime - startTime

    return None, False


  def setKey(self, finalDroneState: "DroneState", isFinalTrajectory:bool=False) -> bool:
    dateString = str(datetime.now()).split(".")[0]
    self.animationKey = f"{dateString} | eps={round(self.cbf.epsilon, 2)} a_max={round(self.cbf.decceleration_max, 2)} | D={finalDroneState.depth} "
    
    if isFinalTrajectory:
      self.animationKey += " | Final trajectory"


  def resetParams(self):
    self.currentAnimationFrame = 0
    self.iterations = 0
    self.movements = []
    self.visitedMovements = set()
    self.currentGoalState = None
    self.currentStartState = None
    self.numIterationsForLastPath = None


def makeAnimationFromTrajectory(planner: "Planner", dt:float=None, lastFrameDuration:int=5000):
  def getImageNum(imageName: str) -> int:
    return int(imageName.split("/")[-1][:-4])

  if dt is None:
    dt = planner.animationFrameDuration * 1000

  images = glob.glob(f"savedTrajectories/{planner.animationKey}/animationFrames/*.png")

  images.sort(key=getImageNum)

  durations = []
  numImages = len(images)

  for _ in range(numImages-1):
    durations.append(dt)

  durations.append(lastFrameDuration)

  imagePath = f"savedTrajectories/{planner.animationKey}/animation.gif"

  frames = [Image.open(image) for image in images]

  frames[0].save(
    imagePath,
    save_all=True, 
    append_images=frames[1:], 
    optimize=False, 
    duration=durations, 
    loop=0
  )


def recordAnimation(initDronePosition: "Position", planner: "Planner", goalStates: List["DroneGoalState"]):
  planner.saveAnimation = True
  numGoalStates = len(goalStates)
  currentDroneState = initDronePosition

  for goalStateIdx in range(numGoalStates):
    currentGoalState = goalStates[goalStateIdx]
    
    initDroneState = DroneState(
        parent=None, 
        x=currentDroneState.x, 
        y=currentDroneState.y,
        yaw=planner.getYawForInitDroneState(currentDroneState, currentGoalState)
      )

    planner.currentStartState = initDroneState

    for i in range(2):
      finalDroneState, foundGoalState = planner.findPathToGoal(initDroneState, currentGoalState)
      currentDroneState = finalDroneState

      if foundGoalState:
        print("Reached the goal! :)")
        
      else:
        print("No path found :(")
        break

      if i == 0:
        planner.setKey(currentDroneState)

        if planner.saveAnimation:
          os.mkdir(f"animationFrames/{planner.animationKey}")

        planner.numIterationsForLastPath = planner.iterations
        planner.iterations = 0
        planner.movements = []
        planner.visitedMovements = set()
      
    if planner.saveAnimation:
      planner.makeAnimationFromTrajectory()
    
    if goalStateIdx < numGoalStates - 1:
      planner.numIterationsForLastPath = None
      planner.currentAnimationFrame = 0
      planner.iterations = 0
      planner.movements = []
      planner.visitedMovements = set()
      
  
  planner.saveAnimation = False


def recordFinalTrajectory(finalDroneState: "DroneState", planner: "Planner", onlyFinalFrame=False, fileName=None):
  planner.saveAnimation = True

  statesInFinalTrajectory = [finalDroneState]

  currentDroneState = finalDroneState.parent

  while currentDroneState is not None:
    statesInFinalTrajectory.append(currentDroneState)
    currentDroneState = currentDroneState.parent
  
  numDroneStatesInFinalTrajectory = len(statesInFinalTrajectory)
  fig, ax = plt.subplots()
  
  if not onlyFinalFrame:
    for droneStateIdx in range(numDroneStatesInFinalTrajectory-1, 0, -1):
      currentDroneState = statesInFinalTrajectory[droneStateIdx]
      planner.plotStateLive(currentDroneState, ax, fig, False)
      planner.currentAnimationFrame += 1

  planner.plotStateLive(statesInFinalTrajectory[0], ax, fig, True, fileName=fileName)

  if not onlyFinalFrame:
    makeAnimationFromTrajectory(planner, dt=planner.dt * 1000, lastFrameDuration=2000)