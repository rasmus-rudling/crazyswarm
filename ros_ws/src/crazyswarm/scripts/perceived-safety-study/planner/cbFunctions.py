from math import sqrt
from matplotlib import pyplot as plt
import numpy as np

import sys
sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/planner')
sys.path.append('/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils')

import seaborn as sns; sns.set_theme()
from plotUtils import saveFig
sns.set(font_scale=2)

class CBF:
  DECELERATION_MAX_MIN = 0.1
  DECELERATION_MAX_MAX = 1.0

  EPSILON_MIN = 0.1
  EPSILON_MAX = 0.75

  VELOCITY_MAX = 1.0

  def __init__(self, decceleration_max, epsilon, key="cbf"):
    self.decceleration_max = decceleration_max
    self.epsilon = epsilon
    self.key = key


  def maxVelocityForCurrentDistance(self, distance):
    if np.isclose(distance, 0) or self.epsilon > distance:
      return 0

    else:
      return np.sqrt(2 * np.abs(self.decceleration_max) * (distance - self.epsilon))


  def velocityIsApprovedForCurrentDistance(self, distance, velocity, droneRadius, humanRadius):
    numerator = velocity ** 2
    denominator = 2 * np.abs(self.decceleration_max)

    minDistance = (numerator/denominator) + self.epsilon

    # return distance >= minDistance  # v1
    return distance >= (minDistance + droneRadius + humanRadius)**2  # v2

def heuristicFunction(distance: float) -> float:
  """The heuristic controll barrier function

  Args:
      distance (float): distance from drone edge to human center 

  Returns:
      velocity (float): maximum allowed velocity in m/s
  """
  if distance < 0.45:
      velocity = 0
  elif 0.45 <= distance and distance < 1.2:
      velocity = 1.0
  elif 1.2 <= distance and distance < 3.6:
      velocity = 2.0
  elif 3.6 <= distance and distance < 7.6:
      velocity = 3.0
  else:
      velocity = 3.5

  return velocity
  

def isApprovedByCbf(distance: float, decelerationMax, epsilon, velocityMax, droneVelocity) -> bool:
  if np.isclose(distance, 0) or epsilon > distance:
    return False

  return np.min([np.sqrt(2 * np.abs(decelerationMax) * (distance - epsilon)), velocityMax]) >= droneVelocity


def cbfForPlotting(distance, decelerationMax, epsilon, velocityMax):
  if np.isclose(distance, 0) or epsilon > distance:
    return 0

  else:
    return np.min([np.sqrt(2 * np.abs(decelerationMax) * (distance - epsilon)), velocityMax])


def plotVelocities():
  totTime = 4
  distances = np.arange(0, totTime, 0.1)

  cbfs = [
    CBF(CBF.DECELERATION_MAX_MIN, CBF.EPSILON_MAX), # Slow breaking + far away from goal => conservative
    CBF(CBF.DECELERATION_MAX_MAX, CBF.EPSILON_MIN), # Hard breaking + close to the goal => liberal
  ]

  for cbf in cbfs:
    maxVelocities = np.array([cbf.maxVelocityForCurrentDistance(distance) for distance in distances])

    plt.rcParams['text.usetex'] = True
    fig, ax = plt.subplots()

    ax.set_xlim(-0.1, totTime)

    fontSize = 25

    ax.set_ylabel(r'velocity (\textit{m/s})', fontsize=fontSize)
    ax.set_ylim(-0.05, CBF.VELOCITY_MAX + 0.25)

    ax.set_xlabel(r'distance (\textit{m})', fontsize=fontSize)


    # function = r'$v_{safe\&fast} = min(v_{max}, \sqrt[+]{2 * | a_{max} | * (d_h - \epsilon)})$'
    # values = r'$v_{max} = $ ' + str(v_max) + r', $a_{max} = $ ' + str(a_max) + r', $\epsilon = $ ' + str(e)
    # ax.set_title(f'Fastest possible velocities based on distance to human that are still safe', fontsize=fontSize)
    sns.lineplot(distances, maxVelocities, marker="o", ax=ax)
    saveFig(fig, f"cbf - a_max={round(cbf.decceleration_max, 2)} | e={round(cbf.epsilon, 2)} | v_max={round(cbf.VELOCITY_MAX, 2)}", 30/25)

  # plt.show()
  
if __name__ == "__main__":
  plotVelocities()