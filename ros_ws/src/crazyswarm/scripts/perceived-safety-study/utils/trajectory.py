import math
from matplotlib import pyplot as plt
from planner.cbFunctions import CBF
from planner.helperFunctions import cmToInches, getDirectionVectorFromAngleAndLength
from planner.obstacle import Obstacle
from position import Position
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from planner.drone import DroneState
sns.set_theme(style="darkgrid")
import csv

class Trajectory:
  def __init__(self, csvFilePath:str=None, finalDroneState:"DroneState"=None, dt:float=0.1):
    """Used for storing, extracting, and visualizing data from a drone trajectory from A ➙ B.

    Args:
        finalDroneState (DroneState): The last drone state in the trajectory.
        dt (float, optional): How much time that passes between each drone state. Defaults to 0.1.
    """
    self.dt = dt
    self.positions = []
    self.x_positions = []
    self.y_positions = []
    self.z_positions = []

    self.velocities = []
    self.x_velocities = []
    self.y_velocities = []
    self.z_velocities = []

    self.accelerations = []
    self.x_accelerations = []
    self.y_accelerations = []
    self.z_accelerations = []

    self.timestamps = []
    self.yaws = []
    self.costs = []

    if finalDroneState is not None:
      self.finalDroneState = finalDroneState
      self.droneStates = [finalDroneState]

      self.saveSimpleDataFromDroneState()
      self.createComplexDataFromDroneState()
    elif csvFilePath is not None:
      self.finalDroneState = None
      self.droneStates = []
      self.extractTrajectoryFromCsv(csvFilePath)

  def setCrazyFlieTrajectory(self, timestamps, positions, velocities, accelerations, yaws, costs):
    self.timestamps= timestamps

    self.positions = positions
    self.x_positions = np.array([position[0] for position in positions]) 
    self.y_positions = np.array([position[1] for position in positions]) 
    self.z_positions = np.array([position[2] for position in positions])

    self.velocities = velocities
    self.x_velocities = np.array([velocity[0] for velocity in velocities])
    self.y_velocities = np.array([velocity[1] for velocity in velocities])
    self.z_velocities = np.array([velocity[2] for velocity in velocities])


    self.accelerations = accelerations
    self.x_accelerations = np.array([acceleration[0] for acceleration in accelerations])
    self.y_accelerations = np.array([acceleration[1] for acceleration in accelerations])
    self.z_accelerations = np.array([acceleration[2] for acceleration in accelerations])
    
    self.yaws = yaws
    self.costs = costs

  def saveSimpleDataFromDroneState(self) -> None:
    """Save the data that is provided by the path planner."""
    def depth(droneState):
      return droneState.depth

    currentDroneState = self.finalDroneState.parent

    while currentDroneState is not None:
      self.droneStates.append(currentDroneState)
      currentDroneState = currentDroneState.parent

    self.droneStates.sort(key=depth) # Make the final drone state last in list

    for currentDroneState in self.droneStates:
      currentPos = Position(currentDroneState.x, currentDroneState.y)
      self.positions.append(currentPos)

      self.velocities.append(currentDroneState.selectedMovement.velocity)
      self.accelerations.append(currentDroneState.selectedMovement.acceleration / self.dt)
      self.yaws.append(currentDroneState.selectedMovement.yaw)
      self.costs.append(currentDroneState.selectedMovement.newDistanceToGoal)
      newTimeStamp = self.timestamps[-1] + self.dt if len(self.timestamps) > 0 else 0
      self.timestamps.append(newTimeStamp)

    self.velocities.insert(0, 0)
    del self.velocities[-1]

  def createComplexDataFromDroneState(self) -> None:
    """Extract axis-specific information about the trajectory."""
    dt = self.timestamps[1]

    for droneStateIdx, currentDroneState in enumerate(self.droneStates):
      x_position, y_position = self.positions[droneStateIdx].x, self.positions[droneStateIdx].y
      self.x_positions.append(x_position)
      self.y_positions.append(y_position)

      currentMovement = currentDroneState.selectedMovement

      isLastState = currentMovement is None

      if not isLastState:
        directionVectorForVelocity = getDirectionVectorFromAngleAndLength(currentMovement.yaw, currentMovement.distance)
        velocityVector = directionVectorForVelocity / dt
        x_velocity, y_velocity = velocityVector
        self.x_velocities.append(x_velocity)
        self.y_velocities.append(y_velocity)

        accelerationDistance = currentMovement.acceleration * dt
        directionVectorForAcceleration = getDirectionVectorFromAngleAndLength(currentMovement.yaw, accelerationDistance)
        accelerationVector = directionVectorForAcceleration / dt
        x_acceleration, y_acceleration = accelerationVector
        self.x_accelerations.append(x_acceleration)
        self.y_accelerations.append(y_acceleration)
      else:
        self.x_velocities.append(None)
        self.y_velocities.append(None)
        self.x_accelerations.append(None)
        self.y_accelerations.append(None)

  def extractTrajectoryFromCsv(self, csvFilePath: str) -> None:
    trajectoryData = np.loadtxt(csvFilePath, delimiter=",", skiprows=1)
      
    self.timestamps = np.array([row[0] for row in trajectoryData])

    self.positions = np.array([row[1:3] for row in trajectoryData])
    self.x_positions = np.array([row[1] for row in trajectoryData])
    self.y_positions = np.array([row[2] for row in trajectoryData])

    self.velocities = np.array([row[3:5] for row in trajectoryData])
    self.x_velocities = np.array([row[3] for row in trajectoryData])
    self.y_velocities = np.array([row[4] for row in trajectoryData])

    self.accelerations = np.array([row[5:7] for row in trajectoryData])
    self.x_accelerations = np.array([row[5] for row in trajectoryData])
    self.y_accelerations = np.array([row[6] for row in trajectoryData])

    self.costs = np.array([row[7] for row in trajectoryData])
    self.yaws = np.array([row[8] for row in trajectoryData])

  def plot(self, filePath: str, cbf:"CBF"=None, show=True, planner=None) -> None:
    def euclidianDist(x1: float, y1: float, x2: float, y2: float) -> float:
      return np.linalg.norm([x1 - x2, y1 - y2])

    def getDistanceToHuman(pos):
      human = planner.HUMAN

      newDistanceToHumanCenter = euclidianDist(pos.x, pos.y, human.x, human.y)
      newDistanceToHuman = newDistanceToHumanCenter - DroneState.DRONE_INFO.radius - human.radius

      return newDistanceToHuman

    """Plot detailed information about the trajectory from start to end.

    Args:
        filePath (str): path to directory where the plot should be stored.
    """
    t = [round(t, 1) for t in self.timestamps]
    
    x = [pos.x for pos in self.positions]
    y = [pos.y for pos in self.positions]

    distancesToHuman = [getDistanceToHuman(pos) for pos in self.positions]
    maxAllowedVelocities = [cbf.maxVelocityForCurrentDistance(d) for d in distancesToHuman]

    data = {
      "Time (s)": t,
      "x (m)": x,
      "y (m)": y,
      "Velocity (m/s)": self.velocities,
      "v_allowed (m/s)": maxAllowedVelocities,
      "Acceleration (m/s²)": self.accelerations,
      "Yaw (°)": np.rad2deg(self.yaws),
      "Cost": self.costs
    }

    df = pd.DataFrame(data)

    fig, ax = plt.subplots(6, sharex=True)

    if planner is not None:
      sns.lineplot(x="Time (s)", y="Velocity (m/s)", data=df, marker="o", ax=ax[0]).set(title=f'Computation time: {planner.computationTime}s')
    else:
      sns.lineplot(x="Time (s)", y="Velocity (m/s)", data=df, marker="o", ax=ax[0])

    sns.lineplot(x="Time (s)", y="v_allowed (m/s)", data=df, marker="o", ax=ax[0])
    sns.lineplot(x="Time (s)", y="Acceleration (m/s²)", data=df, marker="o", ax=ax[1])
    sns.lineplot(x="Time (s)", y="Yaw (°)", data=df, marker="o", ax=ax[2])
    sns.lineplot(x="Time (s)", y="x (m)", data=df, marker="o", ax=ax[3])
    sns.lineplot(x="Time (s)", y="y (m)", data=df, marker="o", ax=ax[4])
    sns.lineplot(x="Time (s)", y="Cost", data=df, marker="o", ax=ax[5])

    ratio = 30 / 18
    width_in_cm = 50
    w = cmToInches(width_in_cm)
    h = cmToInches(width_in_cm / ratio)
    fig.set_size_inches(w, h)
    fig.savefig(filePath, format='png', dpi=144)

    if show:
      plt.show()


  def saveToCsv(self, filePath: str) -> None:
    """Save all necessary information from a trajectory so that it can be reproduced.

    Args:
        filePath (str): path to directory where the csv should be stored.
    """
    with open(filePath, 'w', encoding='UTF8') as f:
      writer = csv.writer(f)
      header = [ 
        "timestamp", 
        "x_position", "y_position", "z_position",
        "yaw",
        # "x_velocity", "y_velocity",
        # "x_acceleration", "y_acceleration", 
        # "cost",  
      ]

      writer.writerow(header)

      delta = 0.1  # In seconds, 0.1s = 10Hz
      t_d = np.arange(0, self.timestamps[-1], delta)
      numDesiredStates = len(t_d)

      x_d = np.interp(t_d, self.timestamps, self.x_positions)
      y_d = np.interp(t_d, self.timestamps, self.y_positions)
      yaw_d = np.interp(t_d, self.timestamps, self.yaws)

      for droneStateIdx in range(numDesiredStates):
        timestamp = t_d[droneStateIdx]
        x_position, y_position = x_d[droneStateIdx], y_d[droneStateIdx]
        yaw = yaw_d[droneStateIdx]

        
        # x_velocity, y_velocity = self.x_velocities[droneStateIdx], self.x_velocities[droneStateIdx]
        # x_acceleration, y_acceleration = self.x_accelerations[droneStateIdx], self.y_accelerations[droneStateIdx]
        # cost = self.costs[droneStateIdx]

        row = [ 
          timestamp, 
          x_position, y_position, -1997,
          yaw,
          # x_velocity, y_velocity,
          # x_acceleration, y_acceleration, 
          # cost,
        ]

        writer.writerow(row)


  def compareWithOtherTrajectory(self, otherTrajectory: "Trajectory") -> "Trajectory":
    trajectoryDiff = Trajectory()

    trajectoryDiff.positions = self.positions - otherTrajectory.positions[:,:2]  # Skip z
    trajectoryDiff.x_positions = self.x_positions - otherTrajectory.x_positions
    trajectoryDiff.y_positions = self.y_positions - otherTrajectory.y_positions
    # trajectoryDiff.z_positions = self.z_positions - otherTrajectory.z_positions

    trajectoryDiff.velocities = self.velocities - otherTrajectory.velocities[:,:2]  # Skip z
    trajectoryDiff.x_velocities = self.x_velocities - otherTrajectory.x_velocities
    trajectoryDiff.y_velocities = self.y_velocities - otherTrajectory.y_velocities
    # trajectoryDiff.z_velocities = self.z_velocities - otherTrajectory.z_velocities

    trajectoryDiff.accelerations = self.accelerations - otherTrajectory.accelerations[:,:2]  # Skip z
    trajectoryDiff.x_accelerations = self.x_accelerations - otherTrajectory.x_accelerations
    trajectoryDiff.y_accelerations = self.y_accelerations - otherTrajectory.y_accelerations
    # trajectoryDiff.z_accelerations = self.z_accelerations - otherTrajectory.z_accelerations

    trajectoryDiff.timestamps = self.timestamps - otherTrajectory.timestamps
    
    trajectoryDiff.yaws = self.yaws - otherTrajectory.yaws
    trajectoryDiff.costs = self.costs - otherTrajectory.costs

    return trajectoryDiff
    
    # TODO: Comment the code
    # TODO: Create scripts that can run the code
    # TODO: Test path with crazyfly simulation
    # TODO: Test with the physical
    # FUTURE TODO: Track people in the room and add them as Humans.