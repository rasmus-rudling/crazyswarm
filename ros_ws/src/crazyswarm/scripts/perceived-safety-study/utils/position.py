from planner.helperFunctions import euclidianDistance

class Position:
  def __init__(self, x: float, y: float) -> None:
    """Stores xy-position

    Args:
        x (float): x-position
        y (float): y-position
    """
    self.x = x
    self.y = y

  def euclidianDistanceToCurrentPosition(self, otherPos: "Position") -> float:
    """Get euclidian distance from other position to current position.

    Args:
        otherPos (Position): check distance to this position.

    Returns:
        float: _description_
    """
    return euclidianDistance(self.x, self.y, otherPos.x, otherPos.y) 

  def reachedCurrentPosition(self, otherPos: "Position") -> bool:
    """Check if current position have reached the "other" position.

    Args:
        otherPos (Position): check if other position have reached current position.

    Returns:
        (bool): True if position is epsilon (0.05) close to other position.
    """
    distance = self.euclidianDistanceToCurrentPosition(otherPos) 

    return distance < 0.05


