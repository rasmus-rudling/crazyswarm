from position import Position

class DroneInfo:
  def __init__(self, radius: float, maxVelocity: float, maxAcceleration: float) -> None:
    """Used to keep track of specs of the drone that will be used.
    Args:
        radius (float): radius of the drone.
        maxVelocity (float): the maximum achievable velocity. TODO
        maxAcceleration (float): the maximum achievable acceleration. TODO
    """
    self.radius = radius
    self.maxVelocity = maxVelocity
    self.maxAcceleration = maxAcceleration


class DroneGoalState(Position):
  def __init__(self, x: float, y: float, radius: float):
    """Used to check if the drone are finished with the trajectory.
    Args:
        x (float): goal x position.
        y (float): goal y position.
        radius (float): basically describes how accurate the drone needs to land at the goal site.
    """
    self.x = x
    self.y = y
    self.radius = radius


class DroneState(Position):
  DRONE_INFO = DroneInfo(radius=0.1, maxVelocity=1.0, maxAcceleration=0.5)

  def __init__(self, parent:"DroneState"=None, x:float=None, y:float=None, yaw:float=None) -> None:
    """Could be seen as a vertex in a graph that exist momentarily.
    Args:
        parent (DroneState, optional): The DroneState whos' movement took the drone to the current state. Defaults to None.
        x (float, optional): current x position. Defaults to None.
        y (float, optional): current y position. Defaults to None.
        yaw (float, optional): current yaw, measured in radians. Defaults to None.
    """
    self.parent = parent
    self.selectedMovement = None

    if parent is not None:
      self.depth = parent.depth + 1
      self.velocity = parent.selectedMovement.velocity
      self.cost = parent.selectedMovement.newDistanceToGoal
      self.x = parent.selectedMovement.newPos.x
      self.y = parent.selectedMovement.newPos.y
      self.yaw = parent.selectedMovement.yaw
    else:
      self.depth = 0
      self.velocity = 0
      self.cost = 0
      self.x = x
      self.y = y
      self.yaw = yaw