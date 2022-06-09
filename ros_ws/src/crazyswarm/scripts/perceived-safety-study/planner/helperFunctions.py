if __name__ == "__main__":
  from movement import Movement

from typing import List, Tuple
import numpy as np


def cmToInches(cm: float) -> float:
    return cm / 2.54


def euclidianDistance(x1: float, y1: float, x2: float, y2: float) -> float:
  # return np.linalg.norm([x1 - x2, y1 - y2])  # v1
  return (x1 - x2)**2 + (y1 - y2)**2  # v2


def getDirectionVectorFromAngleAndLength(angle: float, length: float) -> Tuple[float, float]:
  return np.array([length * np.cos(angle), length * np.sin(angle)])


def printYaw(yawInRad: float) -> None:
  yawInDeg = np.rad2deg(yawInRad)
  print(f"Yaw = {yawInDeg}")


def printDistance(distance: float, text:str="Distance", unit:str="m") -> bool:
  print(f"{text} = {round(distance, 2)}{unit}")


def reverse_insort(movements: List["Movement"], newMovement: "Movement", lo:int=0, hi:int=None) -> None:
    """Insert item newMovement in list movements, and keep it reverse-sorted assuming a
    is reverse-sorted.
    If newMovement is already in movements, insert it to the right of the rightmost newMovement.
    Optional args lo (default 0) and hi (default len(movements)) bound the
    slice of a to be searched.
    """
    numMovements = len(movements)
    if lo < 0:
        raise ValueError('lo must be non-negative')
    if hi is None:
        hi = numMovements
    while lo < hi:
        mid = (lo+hi)//2
        if newMovement.newDistanceToGoal > movements[mid].newDistanceToGoal: hi = mid
        else: lo = mid+1
    movements.insert(lo, newMovement)