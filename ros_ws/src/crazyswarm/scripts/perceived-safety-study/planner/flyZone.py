from typing import List
import unittest
from position import Position

from planner.drone import DroneState

DroneListType = List[DroneState]

class FlyZone:
  def __init__(self, minPos: "Position", maxPos: "Position", padding:int=0):
    self.minPos = minPos
    self.maxPos = maxPos
    self.padding = padding

    if padding != 0:
      minXWithPadding = minPos.x + padding
      minYWithPadding = minPos.y + padding
      minPosWithPadding = Position(minXWithPadding, minYWithPadding)

      maxXWithPadding = maxPos.x - padding
      maxYWithPadding = maxPos.y - padding
      maxPosWithPadding = Position(maxXWithPadding, maxYWithPadding)

      self.flyZoneWithPadding = FlyZone(minPosWithPadding, maxPosWithPadding)
    else:
      self.flyZoneWithPadding = None


class FlyZoneTest(unittest.TestCase):
  minPos = Position(-2.5, -2.0)
  maxPos = Position(2.5, 1.5)
  FLY_ZONE = FlyZone(minPos, maxPos, 0.35)

  def testDroneStatesInsideZone(self):
    dronesInsideOfTheZone = [
      # Above
      DroneState(0.0, 0.7),

      # To the right
      DroneState(1.75, 0.0),

      # Below
      DroneState(0.0, -1.25),

      # To the left
      DroneState(-1.75, -0.5),
    ]

    for drone in dronesInsideOfTheZone:
      droneIsWithinFlyZone = drone.isWithinFlyZone(self.FLY_ZONE)
      self.assertTrue(droneIsWithinFlyZone)

  def testDroneStatesOutsidePadding(self):
    dronesOutsidePadding = [
      # Above
      DroneState(0.0, 1.25),

      # To the right
      DroneState(2.4, 0.0),

      # Below
      DroneState(0.0, -1.75),

      # To the left
      DroneState(-2.1, 0.0),
    ]

    for drone in dronesOutsidePadding:
      droneIsWithinFlyZone = drone.isWithinFlyZone(self.FLY_ZONE)
      self.assertFalse(droneIsWithinFlyZone)

  def testDroneStatesOutsideZone(self):
    dronesOutsideOfTheZone: DroneListType = [
      # Above
      DroneState(-3.0, 2.0),
      DroneState(-1.0, 2.0),
      DroneState(1.0, 2.0),
      DroneState(3.0, 2.0),

      # To the right
      DroneState(3.0, 1.0),
      DroneState(3.0, -1.0),
      DroneState(3.0, -2.0),

      # Below
      DroneState(-3.0, -2.5),
      DroneState(-1.0, -2.5),
      DroneState(3.0, -2.5),
      DroneState(1.0, -2.5),

      # To the left
      DroneState(-3.0, 1.0),
      DroneState(-3.0, -1.0),
      DroneState(-3.0, -2.0),
    ]

    for drone in dronesOutsideOfTheZone:
      droneIsWithinFlyZone = drone.isWithinFlyZone(self.FLY_ZONE)
      self.assertFalse(droneIsWithinFlyZone)

MOCAP_FLY_ZONE = FlyZone(
  minPos=Position(-2.2, -1.97), 
  maxPos=Position(2.3, 1.58), 
  padding=0.2
)

if __name__ == "__main__":
  unittest.main()