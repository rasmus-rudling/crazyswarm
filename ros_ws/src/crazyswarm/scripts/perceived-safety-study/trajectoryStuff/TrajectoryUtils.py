import csv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv

from planner.helperFunctions import euclidianDistance


def cmToInches(cm):
    return cm / 2.54
class TrajectoryUtils:
  def __init__(self, trajectoryDataFile=None, tentacleLength=0.1):
    if trajectoryDataFile is not None:
      trajectoryData = np.loadtxt(trajectoryDataFile, delimiter=",", skiprows=1)
      
      self.timestamps = np.array([row[0] for row in trajectoryData])
      self.positions = np.array([row[1:4] for row in trajectoryData])
      self.velocities = np.array([row[4:7] for row in trajectoryData])
      self.accelerations = np.array([row[7:10] for row in trajectoryData])
      self.yaws = np.array([row[10] for row in trajectoryData])
      self.omegas = np.array([row[11:14] for row in trajectoryData])
    else:
      self.timestamps = np.array([])
      self.positions = np.array([])
      self.velocities = np.array([])
      self.accelerations = np.array([])
      self.yaws = np.array([])
      self.omegas = np.array([])

    self.tentacleLength = tentacleLength


  def appendEvent(self, timestamp, position, yaw=0, velocity=[0, 0, 0], acceleration=[0, 0, 0], omega=[0, 0, 0]):
      self.timestamps = np.append(self.timestamps, timestamp)
      
      # Update position
      old_pos = self.positions
      new_pos = np.array([position])
      self.positions = new_pos if len(old_pos) == 0 else np.append(old_pos, new_pos, axis=0)

      # Update velocities
      old_vel = self.velocities
      new_vel = np.array([velocity])
      self.velocities = new_vel if len(old_vel) == 0 else np.append(old_vel, new_vel, axis=0)

      # Update accelerations
      old_acc = self.accelerations
      new_acc = np.array([acceleration])
      self.accelerations = new_acc if len(old_acc) == 0 else np.append(old_acc, new_acc, axis=0)

      self.yaws = np.append(self.yaws, yaw)

      # Update omegas
      old_omegas = self.omegas
      new_omegas = np.array([omega])
      self.omegas = new_omegas if len(old_omegas) == 0 else np.append(old_omegas, new_omegas, axis=0)


  def appendDroneEvent(self, timestamp, drone):
    # print(f"Curr: {drone.position()}\n")
    self.appendEvent(
      timestamp=timestamp, 
      position=drone.position(), 
      velocity=drone.velocity(), 
      acceleration=drone.acceleration(), 
      yaw=drone.yaw(), 
      omega=drone.setState.omega
    )


  def saveTrajectoryToCsv(self, fileName):
    with open(fileName, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            header = [ 
              "timestamp", 
              "pos_x", "pos_y", "pos_z", 
              "x_vel", "y_vel", "z_vel",
              "x_acc", "y_acc", "z_acc", 
              "yaw", 
              "roll_vel", "pitch_vel", "yaw_vel"
            ]

            writer.writerow(header)

            numEvents = len(self.timestamps)
            for i in range(numEvents):
                timestamp = self.timestamps[i]
                pos = self.positions[i]
                pos_x, pos_y, pos_z = pos[0], pos[1], pos[2] 
                x_vel, y_vel, z_vel = self.velocities[i][0], self.velocities[i][1], self.velocities[i][2] 
                x_acc, y_acc, z_acc = self.accelerations[i][0], self.accelerations[i][1], self.accelerations[i][2] 
                yaw = self.yaws[i]
                omega = self.omegas[i]
                roll_vel, pitch_vel, yaw_vel = omega[0], omega[1], omega[2]

                row = [ timestamp, 
                        pos_x, pos_y, pos_z, 
                        x_vel, y_vel, z_vel,
                        x_acc, y_acc, z_acc, 
                        yaw, 
                        roll_vel, pitch_vel, yaw_vel
                ]

                writer.writerow(row)


  def compareWithOtherTrajectory(self, otherTrajectory):
    trajectoryDiff = TrajectoryUtils()

    trajectoryDiff.timestamps = self.timestamps
    trajectoryDiff.positions = self.positions - otherTrajectory.positions
    trajectoryDiff.velocities = self.velocities - otherTrajectory.velocities
    trajectoryDiff.accelerations = self.accelerations - otherTrajectory.accelerations
    trajectoryDiff.yaws = self.yaws - otherTrajectory.yaws
    trajectoryDiff.omegas = self.omegas - otherTrajectory.omegas

    font = {'size': 30}
    matplotlib.rc('font', **font)

    fig, axs = plt.subplots(3)

    # --- PLOT 1 ---

    axs[0].plot(trajectoryDiff.timestamps, trajectoryDiff.velocities[:, 0])
    axs[0].plot(trajectoryDiff.timestamps, trajectoryDiff.velocities[:, 1])
    axs[0].plot(trajectoryDiff.timestamps, trajectoryDiff.velocities[:, 2])

    legend0 = axs[0].legend(('x', 'y', 'z'), loc=(
        0.02, 0.7), prop={'size': 30})

    for legendObj in legend0.legendHandles:
        legendObj.set_linewidth(4.2)

    axs[0].set(
      xlabel='time (s)', ylabel='velocity (m/s)',
      title='Velocity diff between desired trajectory and executed trajectory'
    )
    axs[0].grid()

    # --- PLOT 2 ---

    axs[1].plot(trajectoryDiff.timestamps, trajectoryDiff.positions[:, 0])
    axs[1].plot(trajectoryDiff.timestamps, trajectoryDiff.positions[:, 1])
    axs[1].plot(trajectoryDiff.timestamps, trajectoryDiff.positions[:, 2])

    legend1 = axs[1].legend(('x', 'y', 'z'), loc=(
        0.02, 0.7), prop={'size': 30})

    for legendObj in legend1.legendHandles:
        legendObj.set_linewidth(4.2)

    axs[1].set(
      xlabel='time (s)', ylabel='distance (m)',
      title='Position diff between desired trajectory and executed trajectory'
    )

    axs[1].grid()

    # --- PLOT 3 ---
    axs[2].plot(trajectoryDiff.timestamps, trajectoryDiff.accelerations[:, 0])
    axs[2].plot(trajectoryDiff.timestamps, trajectoryDiff.accelerations[:, 1])
    axs[2].plot(trajectoryDiff.timestamps, trajectoryDiff.accelerations[:, 2])

    legend1 = axs[2].legend(('x', 'y', 'z'), loc=(
        0.02, 0.7), prop={'size': 30})

    for legendObj in legend1.legendHandles:
        legendObj.set_linewidth(4.2)

    axs[2].set(
      xlabel='time (s)', ylabel='acceleration (m/s²)',
      title='Acceleration diff between desired trajectory and executed trajectory'
    )

    # axs[2].grid()

    # --------------

    ratio = 30 / 30
    width_in_inches = 65
    w = cmToInches(width_in_inches)
    h = cmToInches(width_in_inches / ratio)
    fig.set_size_inches(w, h)

    filename = "diffBetweenTrajectoryAndLoggedTrajectory"
    fig.savefig(f"plots/{filename}.png", format='png', dpi=72)

    font = {'size': 12}
    matplotlib.rc('font', **font)

    trajectoryDiff.saveTrajectoryToCsv(f"csvs/{filename}.csv")


class Obstacle:
  def __init__(self, x, y, radius):
    self.x = x
    self.y = y
    self.radius = radius

class Position:
  def __init__(self, x, y):
    self.x = x
    self.y = y

  def euclidianDistanceToCurrentPosition(self, otherPos):
    return euclidianDistance(self.x, self.y, otherPos.x, otherPos.y) 

  def reachedCurrentPosition(self, otherPos, currentPlanningState):
    distance = self.euclidianDistanceToCurrentPosition(otherPos) 

    # print(f"D#{currentPlanningState.depth} | Distance = {round(distance, 2)}m")

    return distance < 0.05

  def collidedWithObstacle(self, obstacle):
    distanceToObstacleCenter = self.euclidianDistanceToCurrentPosition(obstacle)

    haveCollided = distanceToObstacleCenter <= obstacle.radius
    return haveCollided

class PlanningState:
  def __init__(self, parent, depth, position, yaw):
    self.parentState = parent
    self.depth = depth
    self.position = position
    self.yaw = yaw
    self.tried = {
      "left": False,
      "straight": False,
      "right": False
    }
    self.child = None

class LatticePath:
  def __init__(self, x, y, direction):
    self.x = x
    self.y = y
    self.direction = direction
    self.tentacleLength = euclidianDistance(0, 0, x[-1], y[-1])

    startPos = np.array([0, 0])
    goalPos = np.array([x[-1], y[-1]])
    self.hypotenuse = np.linalg.norm(startPos - goalPos)

    if direction == "left":
      self.angle = np.arccos(x[-1] / self.hypotenuse)
    elif direction == "straight":
      self.angle = 0
    elif direction == "right":
      self.angle = - np.arccos(x[-1] / self.hypotenuse)

    # print(f"({self.direction[0]}) Angle = {np.rad2deg(self.angle)}°")

  def getNewPos(self, currentYaw, currentPos):
    endAngle = currentYaw + self.angle

    return Position(currentPos.x + self.hypotenuse * np.cos(endAngle), currentPos.y + self.hypotenuse * np.sin(endAngle)), endAngle


  def plotLatticePath(self):
    fig, ax = plt.subplots()
    ax.plot(self.x, self.y)

    ax.set(xlabel='x', ylabel='y',
          title=f'Tentacle {self.direction}')
    ax.grid()

    plt.show()


class TrajectoryPlanner:
  def __init__(self, latticePaths, obstacle=Obstacle(0, 0, 2), dt=0.1, startYaw=0, startX=1.5, startY=1.5, goalX=-3, goalY=-3):
    self.leftLatticePaths = latticePaths[0]
    self.straightLatticePaths = latticePaths[1]
    self.rightLatticePaths = latticePaths[2]
    self.startYaw = startYaw
    self.startPos = Position(startX, startY)
    self.goalPos = Position(goalX, goalY)
    self.rootPlanningState = PlanningState(None, 0, self.startPos, self.startYaw)
    self.dt = dt
    self.obstacle = obstacle
    self.visitedStates = {}


  def findPathToGoal(self):
    # TODO: Rewrite to make code easier to understand
    
    foundGoal = False
    parentPlanningState = self.rootPlanningState
    collidedWithObstacle = False
    selectedLatticePath = LatticePath([-1], [-1], "start")
    newPos = parentPlanningState.position
    newYaw = parentPlanningState.yaw
    currentPlanningState = None

    while not foundGoal:
      stateAlreadyVisited = currentPlanningState is not None and str([newPos.x, newPos.y, newYaw]) in self.visitedStates

      if collidedWithObstacle or selectedLatticePath is None or stateAlreadyVisited:
        if stateAlreadyVisited:
          newState = currentPlanningState
        elif currentPlanningState.parentState is not None:
          newState = currentPlanningState.parentState
        else:
          x = currentPlanningState.position.x
          y = currentPlanningState.position.y
          yaw = currentPlanningState.yaw
          isRootState = x == self.startPos.x and y == self.startPos.y and yaw == self.startYaw

          stateKey = str([currentPlanningState.position.x, currentPlanningState.position.y, currentPlanningState.yaw])
          stateExist = stateKey in self.visitedStates
          stateAlreadyVisited = currentPlanningState.tried["left"] and currentPlanningState.tried["straight"] and currentPlanningState.tried["right"]

          if isRootState and stateAlreadyVisited:
              print(f"isRootState={isRootState}, stateAlreadyVisited={stateAlreadyVisited}")
              print("No path found! :(")
              exit(0) 

        while stateAlreadyVisited:
          # print(f"newState (x, y, yaw)=({newState.position.x},{newState.position.y},{newState.yaw})")
          if newState.parentState is not None:
            newState = newState.parentState
          stateKey = str([newState.position.x, newState.position.y, newState.yaw])
          stateExist = stateKey in self.visitedStates

          if stateExist:
            state = self.visitedStates[stateKey]
            stateAlreadyVisited = state.tried["left"] and state.tried["straight"] and state.tried["right"]

            x = state.position.x
            y = state.position.y
            yaw = state.yaw

            isRootState = x == self.startPos.x and y == self.startPos.y and yaw == self.startYaw
            
            if isRootState and stateAlreadyVisited:
              print(f"isRootState={isRootState}")
              print("No path found! :(")
              exit(0) 
            elif isRootState:
              print(f"isRootState={isRootState}")
              stateAlreadyVisited = False

        currentPlanningState = newState
      else:
        # print("Completely new state")
        currentPlanningState = PlanningState(parentPlanningState, parentPlanningState.depth + 1, newPos, newYaw)
        stateKey = str([currentPlanningState.position.x, currentPlanningState.position.y, currentPlanningState.yaw])
        self.visitedStates[stateKey] = currentPlanningState

      selectedLatticePath = self.selectLatticePath(currentPlanningState)

      if selectedLatticePath is None:
        # print("No path found")
        continue

      currentPlanningState.tried[selectedLatticePath.direction] = True
      
      newPos, newYaw = selectedLatticePath.getNewPos(currentPlanningState.yaw, currentPlanningState.position)

      collidedWithObstacle = newPos.collidedWithObstacle(self.obstacle)

      if collidedWithObstacle:
        # print("Collided!")
        continue

      parentPlanningState = currentPlanningState
      foundGoal = self.goalPos.reachedCurrentPosition(currentPlanningState.position, currentPlanningState)

      if foundGoal:
        print("\nFound the goal! :)\n")

    finalX = []
    finalY = []
    finalYaw = []

    while currentPlanningState is not None:
      finalX.insert(0, currentPlanningState.position.x)
      finalY.insert(0, currentPlanningState.position.y)
      finalYaw.insert(0, currentPlanningState.yaw)
      currentPlanningState = currentPlanningState.parentState

    trajectoryToGoal = TrajectoryUtils()
    
    numEvents = len(finalX)

    for event_idx in range(numEvents):
      timestamp = event_idx * self.dt
      x = finalX[event_idx]
      y = finalY[event_idx]
      position = [x, y, -1]
      yaw = finalYaw[event_idx]

      trajectoryToGoal.appendEvent(timestamp, position, yaw)
      # trajectoryToGoal.saveTrajectoryToCsv("../csvs/PathPlanningTrajectory.csv")

    return trajectoryToGoal


  def selectLatticePath(self, currentPlanningState):
    leftEndPos = self.leftLatticePaths.getNewPos(currentPlanningState.yaw, currentPlanningState.position)[0]
    straightEndPos = self.straightLatticePaths.getNewPos(currentPlanningState.yaw, currentPlanningState.position)[0]
    rightEndPos = self.rightLatticePaths.getNewPos(currentPlanningState.yaw, currentPlanningState.position)[0]

    # print(f"Left end position:     [x, y]=[{leftEndPos.x}, {leftEndPos.y}]")
    # print(f"Straight end position: [x, y]=[{straightEndPos.x}, {straightEndPos.y}]")
    # print(f"Right end position:    [x, y]=[{rightEndPos.x}, {rightEndPos.y}]")

    leftDistanceToGoal = self.goalPos.euclidianDistanceToCurrentPosition(leftEndPos)
    straightDistanceToGoal = self.goalPos.euclidianDistanceToCurrentPosition(straightEndPos)
    rightDistanceToGoal = self.goalPos.euclidianDistanceToCurrentPosition(rightEndPos)
    
    distances = {}

    distances[leftDistanceToGoal] = self.leftLatticePaths
    distances[straightDistanceToGoal] = self.straightLatticePaths
    distances[rightDistanceToGoal] = self.rightLatticePaths

    # print(f"Left distance to goal     = {leftDistanceToGoal}m")
    # print(f"Straight distance to goal = {straightDistanceToGoal}m")
    # print(f"Right distance to goal    = {rightDistanceToGoal}m")
    

    sortedDistances = sorted([leftDistanceToGoal, straightDistanceToGoal, rightDistanceToGoal])

    selectedLattice = None

    for distance in sortedDistances:
      latticePath = distances[distance]
      haveTriedPath = currentPlanningState.tried[latticePath.direction]

      if not haveTriedPath:
        selectedLattice = latticePath
        break


    # if selectedLattice is not None:
    #   print(f"\n{selectedLattice.direction} lattice selected\n")

    return selectedLattice


def main():
  pass


if __name__ == "__main__":
  main()