if __name__ == "__main__":  # To avoid circular import
  from planner import Planner

import numpy as np
from planner.drone import DroneState
from planner.helperFunctions import getDirectionVectorFromAngleAndLength
from position import Position

EPSILON_CLOSE_TO_GOAL = 0.2

class Movement:

  POSSIBLE_ANGLES = [
    np.deg2rad(-5.0), 
    np.deg2rad(0.0), 
    np.deg2rad(5.0)
  ]
  EPSILON = 0
  VELOCITY_MAX = 1.0

  def __init__(self, angle: float, acceleration: float, planner: "Planner", associatedDroneState: "DroneState") -> None:
    self.EPSILON = planner.HUMAN.radius

    self.angle = angle
    self.acceleration = acceleration

    self.planner = planner
    self.associatedDroneState = associatedDroneState

    self.velocity = self.associatedDroneState.velocity + self.acceleration
    self.yaw = self.associatedDroneState.yaw + self.angle
    self.distance = self.velocity * planner.dt
    
    self.newPos = self.getNewPos()
    self.newDistanceToHuman = self.getNewDistanceToHuman()
    self.newDistanceToGoal = self.getNewDistanceToGoal()
    self.isValid = self.getMovementIsValid()

    # if self.isValid:
    self.willHaveReachedGoal = self.willHaveReachedGoal()
    

    self.key = f"{self.angle} | {self.acceleration} | ({self.associatedDroneState.x}, {self.associatedDroneState.y}) | {self.velocity} | {self.yaw} | {self.newPos} | {self.newDistanceToGoal} | {self.newDistanceToGoal}"

  def willHaveReachedGoal(self, epsilon:float=EPSILON_CLOSE_TO_GOAL) -> bool:
    isEpsilonCloseToGoal = self.newDistanceToGoal <= epsilon**2
    isEpsilonCloseToZeroVelocity = 0 <= self.velocity <= epsilon

    if not isEpsilonCloseToGoal or not isEpsilonCloseToZeroVelocity:
      return False

    if self.planner.cbf.key == "cbf":
      approvedByCbf = self.planner.cbf.velocityIsApprovedForCurrentDistance(
        self.newDistanceToHuman, 
        self.velocity,
        droneRadius=self.associatedDroneState.DRONE_INFO.radius,
        humanRadius=self.planner.HUMAN.radius
      )
    else:
      maxVelocityForCurrentDistanceToHuman = self.planner.cbf(self.newDistanceToHuman)
    
    willHaveReachedGoal = approvedByCbf and isEpsilonCloseToGoal and isEpsilonCloseToZeroVelocity

    return willHaveReachedGoal 

  def getNewDistanceToGoal(self) -> float:
    return self.newPos.euclidianDistanceToCurrentPosition(self.planner.currentGoalState)


  def getNewDistanceToHuman(self) -> float:
    return self.newPos.euclidianDistanceToCurrentPosition(self.planner.HUMAN)


  def getMovementIsValid(self) -> bool:
    if self.planner.cbf.key == "cbf":
      approvedByCbf = self.planner.cbf.velocityIsApprovedForCurrentDistance(
        self.newDistanceToHuman, 
        self.velocity,
        droneRadius=self.associatedDroneState.DRONE_INFO.radius,
        humanRadius=self.planner.HUMAN.radius
      )
    else:
      maxVelocityForCurrentDistanceToHuman = self.planner.cbf(self.newDistanceToHuman)

    droneIsNotStandingStill = self.velocity >= 0.001

    movementIsValid = approvedByCbf and droneIsNotStandingStill
    return movementIsValid


  def getNewPos(self) -> "Position":
    newPos = np.array([self.associatedDroneState.x, self.associatedDroneState.y]) + getDirectionVectorFromAngleAndLength(self.yaw, self.distance)

    return Position(newPos[0], newPos[1])


  def willBeWithinFlyZone(self) -> bool:
    if self.planner.flyZone.flyZoneWithPadding is not None:
      flyZone = self.planner.flyZone.flyZoneWithPadding 
    else:
      flyZone = self.planner.flyZone

    distanceToTop = flyZone.maxPos.y - self.newPos.y
    distanceToBottom = self.newPos.y - flyZone.minPos.y
    distanceToRight = flyZone.maxPos.x - self.newPos.x
    distanceToLeft = self.newPos.x - flyZone.minPos.x

    distances = [distanceToTop, distanceToBottom, distanceToRight, distanceToLeft]
    droneRadius = self.associatedDroneState.DRONE_INFO.radius

    for distance in distances:
      if distance < droneRadius:
        return False
    
    return True


  def willCollideWithObstacle(self) -> bool:
    for obstacle in self.planner.obstacles:
      distanceToObstacleCenter = self.newPos.euclidianDistanceToCurrentPosition(obstacle)

      if distanceToObstacleCenter <= (obstacle.radius + self.associatedDroneState.DRONE_INFO.radius):
        return True

    return False 