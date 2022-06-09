from cmath import asin
import math
import os
import sys

from matplotlib import pyplot as plt
import numpy as np
from SimpleTrajectory import Z_HEIGHT, SimpleTrajectory
import rospy
from trajectoryUtils import getLatestTrajectory
sys.path.append("..")
from pycrazyswarm import Crazyswarm
from tf import TransformListener
from tf import transformations


class TestData:
  def __init__(self):
    self.currentPose = Pose(0, 0, 0, yaw=0)

class Quaternion:
  def __init__(self, x, y, z, w):
    self.x = x
    self.y = y
    self.z = z
    self.w = w

  def __str__(self):
    return f"-Quaternion-\nx={self.x}\ny={self.y}\nz={self.z}\nw={self.w}\n"

class Pose:
  def __init__(self, x, y, z, yaw = None, pitch = None, roll = None):
      self.x = x
      self.y = y
      self.z = z

      self.pitch = pitch
      self.roll = roll
      self.yaw = yaw
      
  def setPitchRollYaw(self, q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    self.roll, self.pitch, self.yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])


  def __str__(self):
    return f"-Pose-\nx={self.x}\ny={self.y}\nz={self.z}\nroll={self.roll}\npitch={self.pitch}\nyaw={self.yaw}\n"


class CrazyflieController:
  def __init__(self, testData = None):
    if testData is None:
      self.testData = None
      crazyflies_yaml = str({'crazyflies': [{'channel': 100, 'id': 7, 'initialPosition': [0, 0, 0], 'type': 'default'}]})
      swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
      self.timeHelper = swarm.timeHelper
      self.drone = swarm.allcfs.crazyflies[0]
    else:
      self.testData = testData

    self.delta = 1 / 10 # 0.1
    # self.delta = 1 / 20 # 0.05

    self.baseVelocity = 1.5

  def land(self, velocity = None):
    if velocity is None:
      velocity = self.baseVelocity

    landingHeight = 0.05
    currentPose = self.getCurrentPose()

    landedPose = Pose(currentPose.x, currentPose.y, landingHeight, yaw=currentPose.yaw)

    landingTrejectory = self.getTrajectoryToPose(landedPose)
    landingTrejectory.slowTrajectory(factor=4)

    recordedPosesDuringLanding, recordedTrajectoryDuringLanding = self.followTrajectory(landingTrejectory)

    return recordedPosesDuringLanding, recordedTrajectoryDuringLanding


  def hover(self, duration):
    currentPose = self.getCurrentPose()

    x, y, z = currentPose.x, currentPose.y, currentPose.z
    position = np.array([x, y, z])

    recordedPosesDuringHover = []

    hoverLoopRange = np.arange(0, duration, self.delta)

    timestamps = []

    for timestamp in hoverLoopRange:
      timestamps.append(timestamp)

      self.drone.cmdPosition(position, currentPose.yaw)
      self.timeHelper.sleep(self.delta)

      currentDronePose = self.getCurrentPose()
      recordedPosesDuringHover.append(currentDronePose)

    recordedTrajectory = SimpleTrajectory(timestamps=timestamps, poses=recordedPosesDuringHover)
    return recordedPosesDuringHover, recordedTrajectory

  def getCurrentPose(self):
    tf = TransformListener()
    tf.waitForTransform("/world", "/cf7", rospy.Time(), rospy.Duration(4.0))

    t = tf.getLatestCommonTime("/world", "/cf7")
    initPosition, q = tf.lookupTransform('/world', "/cf7", t)

    quaternion = Quaternion(q[0], q[1], q[2], q[3])
    x, y, z = initPosition

    currentDronePose = Pose(x, y, z)
    currentDronePose.setPitchRollYaw(quaternion)

    return currentDronePose


  def followTrajectory(self, trajectory, reverse = False, error = None, delta=None):
    if delta is None:
      delta = self.delta

    recordedPoses = []
    numStates = trajectory.timestamps.shape[0]

    trajectoryRange = range(numStates-1, -1, -1) if reverse else range(numStates)

    for i in trajectoryRange:
      xTarget, yTarget, zTarget = trajectory.x[i], trajectory.y[i], trajectory.z[i]

      currentTargetPosition = np.array([xTarget, yTarget, zTarget])

      if error is not None:
        currentTargetPosition += error

      yawTarget = trajectory.yaw[i]
      self.drone.cmdPosition(currentTargetPosition, yawTarget)
      self.timeHelper.sleep(delta)

      currentDronePose = self.getCurrentPose()
      recordedPoses.append(currentDronePose)

    recordedTrajectory = SimpleTrajectory(timestamps=trajectory.timestamps, poses=recordedPoses)
    return recordedPoses, recordedTrajectory


  def getTrajectoryToPose(self, goalPose, velocity = None):
    if velocity is None:
      velocity = self.baseVelocity
    
    if self.testData is not None:
      currentPose = self.testData.currentPose
    else:
      currentPose = self.getCurrentPose()

    currentPosition = np.array([currentPose.x, currentPose.y, currentPose.z])
    goalPosition = np.array([goalPose.x, goalPose.y, goalPose.z])
    distanceToGoalPosition = np.linalg.norm(currentPosition-goalPosition)
    duration = distanceToGoalPosition / velocity

    numValues = int(np.round(duration / self.delta))

    # Make sure all axes (and yaw) get same number of values.
    # If x needs to turn more than y, x will turn faster than y, 
    # but they will both have the same number of values
    x = np.linspace(currentPose.x, goalPose.x, numValues)
    y = np.linspace(currentPose.y, goalPose.y, numValues)
    z = np.linspace(currentPose.z, goalPose.z, numValues)
    yaw = np.linspace(currentPose.yaw, goalPose.yaw, numValues)
    timesteps = np.arange(0, duration, self.delta)

    minNumValues = np.min([x.shape[0], y.shape[0], z.shape[0], yaw.shape[0], timesteps.shape[0]])
    x = x[:minNumValues]
    y = y[:minNumValues]
    z = z[:minNumValues]
    yaw = yaw[:minNumValues]
    timesteps = timesteps[:minNumValues]

    # "Simple" trajectory to not confuse with the trajectory created by the motion planner
    return SimpleTrajectory(timesteps, x, y, z, yaw)


def flight1(droneController: "CrazyflieController"):
  plannedTrajectory, pathToTrajectoryFolder = getLatestTrajectory()
  plannedTrajectory = plannedTrajectory.interpolateTrajectory(0.05)

  x_start, y_start = plannedTrajectory.x[0], plannedTrajectory.y[0]
  z_start = 1.5
  yaw_start = plannedTrajectory.yaw[0]
  startingPose = Pose(x_start, y_start, z_start, yaw=yaw_start)

  trajetoryToStartPose = droneController.getTrajectoryToPose(startingPose, velocity=0.5)
  droneController.followTrajectory(trajetoryToStartPose)
  droneController.hover(5)

  # TODO: Be able to plot the trajectory even though it crashes
  # droneController.followTrajectory(plannedTrajectory)
  droneController.land(0.5)

  # droneController.followTrajectory(trajetoryToStartPose, reverse=True)
  

def flight2(droneController: "CrazyflieController"):
  landAndHovertrajectory = SimpleTrajectory(csv="hoverAndLandDataInMeters.csv")
  interpolatedTrajectory = landAndHovertrajectory.interpolateTrajectory(0.05)
  
  x_start_recorded = interpolatedTrajectory.x[0]
  y_start_recorded = interpolatedTrajectory.y[0]
  z_start_recorded = interpolatedTrajectory.z[0]

  currentDronePose = droneController.getCurrentPose()
  starting_offset = np.array([currentDronePose.x, currentDronePose.y, currentDronePose.z]) - np.array([x_start_recorded, y_start_recorded, z_start_recorded])
  
  recordedPoses = droneController.followTrajectory(interpolatedTrajectory, error=starting_offset)
  recordedTrajectory = SimpleTrajectory(timestamps=interpolatedTrajectory.timestamps, poses=recordedPoses)

  interpolatedTrajectory.plotTrajectory(recordedTrajectory)


def flight3(droneController: "CrazyflieController"):
  goalPose = Pose(0, 0, 1.5, np.deg2rad(180))
  trajecoryToHoverPose = droneController.getTrajectoryToPose(goalPose, velocity=0.5)
  recordedTrajectory = droneController.followTrajectory(trajecoryToHoverPose)[1]
  droneController.hover(2)

  goalPose = Pose(-1.5, 1, 1.5, np.deg2rad(30))
  trajecoryToGoal = droneController.getTrajectoryToPose(goalPose, velocity=1.25)
  recordedTrajectory = droneController.followTrajectory(trajecoryToGoal)[1]
  droneController.hover(3.5)

  droneController.land(0.5)
  trajecoryToGoal.plotTrajectory(otherTrajectory=recordedTrajectory)


def flight4(droneController: "CrazyflieController"):
  plannedTrajectory, pathToTrajectoryFolder = getLatestTrajectory()
  goalPose = Pose(plannedTrajectory.x[0], plannedTrajectory.y[0], Z_HEIGHT, plannedTrajectory.yaw[0])

  trajecoryToHoverPose = droneController.getTrajectoryToPose(goalPose, velocity=0.5)
  recordedTrajectory = droneController.followTrajectory(trajecoryToHoverPose)[1]
  droneController.hover(2)

  recordedTrajectory = droneController.followTrajectory(plannedTrajectory)[1]
  droneController.hover(3.5)

  droneController.land(0.5)
  plannedTrajectory.plotTrajectory(otherTrajectory=recordedTrajectory, fileName=f"{pathToTrajectoryFolder}/executedTrajectory")


def flight5(droneController: "CrazyflieController"):
  goalPose = Pose(0, 0, 1.5, np.deg2rad(180))
  trajecoryToHoverPose = droneController.getTrajectoryToPose(goalPose, velocity=0.5)

  trajecoryToHoverPose.visualize()

  recordedTrajectory = droneController.followTrajectory(trajecoryToHoverPose)[1]
  droneController.hover(duration=1)
  droneController.land(velocity=0.5)

  trajecoryToHoverPose.plotTrajectory(otherTrajectory=recordedTrajectory)


def flight6(droneController: "CrazyflieController"):
  goalPose = Pose(0, 0, 1.5, np.deg2rad(180))
  trajecoryToHoverPose = droneController.getTrajectoryToPose(goalPose, velocity=0.5)

  recordedTrajectory = droneController.followTrajectory(trajecoryToHoverPose)[1]
  droneController.hover(duration=1)
  droneController.land(velocity=0.5)

  trajecoryToHoverPose.plotTrajectory(otherTrajectory=recordedTrajectory)


if __name__ == "__main__":
  flights = {
    "f1": flight1,
    "f2": flight2,
    "f3": flight3,
    "f4": flight4,
    "f5": flight5,
    "f6": flight6,
  }

  if len(sys.argv) > 2 and sys.argv[2] == "t":
    droneController = CrazyflieController(testData = TestData())
  else:
    droneController = CrazyflieController()

  flights[sys.argv[1]](droneController)