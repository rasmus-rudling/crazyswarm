from math import sqrt
from time import sleep
from matplotlib import pyplot as plt
import numpy as np

import sys

sys.path.append(
    '/home/rpl/Documents/rasmus/crazyswarm/ros_ws/src/crazyswarm/scripts/perceived-safety-study/planner'
)
sys.path.append(
    '/home/rpl/Documents/rasmus/crazyswarm/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils'
)

from globalVariables import DRONE_MAX_VELOCITY
import seaborn as sns

sns.set_theme()
from plotUtils import saveFig

sns.set(font_scale=2)


class CBF:
    DECELERATION_MAX_MIN = 0.1
    DECELERATION_MAX_MAX = 1.0

    EPSILON_MIN = 0.1
    EPSILON_MAX = 0.75

    def __init__(self, decceleration_max, epsilon):
        self.name = "cbf"

        self.decceleration_max = decceleration_max
        self.epsilon = epsilon

    def maxVelocityForCurrentDistance(self, distance):
        if np.isclose(distance, 0) or self.epsilon > distance:
            return 0

        else:
            return np.min([
                np.sqrt(2 * np.abs(self.decceleration_max) *
                        (distance - self.epsilon)), DRONE_MAX_VELOCITY
            ])

    def velocityIsApprovedForCurrentDistance(self, distance, velocity,
                                             droneRadius, humanRadius):
        numerator = velocity**2
        denominator = 2 * np.abs(self.decceleration_max)

        minDistance = (numerator / denominator) + self.epsilon

        return distance >= (minDistance + droneRadius +
                            humanRadius)**2 and velocity <= DRONE_MAX_VELOCITY


class HeuristicSafetyFunction:
    def __init__(self, cbf: "CBF" = None):
        self.name = "heuristic"
        self.zoneLimits = np.array([0.45, 1.2, 3.6, 7.6])

        if cbf is None:
            # self.velocitiesWithinZone = np.array([0, 0.75, 1.5, 1.5])
            cbf = CBF(0.1, 0.1)

        maxVelocities = [0]

        for d in self.zoneLimits:
            maxV = cbf.maxVelocityForCurrentDistance(d)
            maxVelocities.append(maxV)

        self.velocitiesWithinZone = np.array(maxVelocities[:-1])
        self.zones = {}

        for i, zoneLimit in enumerate(self.zoneLimits):
            self.zones[zoneLimit] = self.velocitiesWithinZone[i]

    def maxVelocityForCurrentDistance(self, distance: float) -> float:
        """The heuristic controll barrier function

    Args:
        distance (float): distance from drone edge to human center 

    Returns:
        velocity (float): maximum allowed velocity in m/s
    """

        distance += 0.25

        diffs = self.zoneLimits - distance

        for i, diff in enumerate(diffs):
            if diff > 0:
                return self.velocitiesWithinZone[i]

        return self.velocitiesWithinZone[-1]

    def velocityIsApprovedForCurrentDistance(self, distance, velocity,
                                             droneRadius, humanRadius):
        # reversedLimits = sorted(self.zoneLimits, reverse=True)
        # reversedVelocities = np.array(sorted(self.velocitiesWithinZone, reverse=True))

        diffs = self.velocitiesWithinZone - velocity
        minDistance = 99999999

        for i, diff in enumerate(diffs):
            if diff > 0:
                minDistance = self.zoneLimits[i - 1]
                break

        # appr = distance >= (minDistance + droneRadius)**2 and velocity <= DRONE_MAX_VELOCITY
        # print(f"{round(velocity, 2)} m/s -> min. D = {minDistance} m | Approved if {round(distance, 2)} ≥ {round((minDistance + droneRadius)**2, 2)} -> {'Approved' if appr else 'Denied'}")

        # sleep(0.25)

        return distance >= (minDistance +
                            droneRadius)**2 and velocity <= DRONE_MAX_VELOCITY

    def plot(self):
        distances = np.arange(0, 3, 0.02)
        v = np.array(
            [self.maxVelocityForCurrentDistance(d) for d in distances])

        plt.plot(distances, v)
        plt.show()
        plt.pause(9999)


def plotVelocities():
    totTime = 4
    distances = np.arange(0, totTime, 0.1)

    cbfs = [
        CBF(CBF.DECELERATION_MAX_MIN, CBF.EPSILON_MAX
            ),  # Slow breaking + far away from goal => conservative
        CBF(CBF.DECELERATION_MAX_MAX,
            CBF.EPSILON_MIN),  # Hard breaking + close to the goal => liberal
    ]

    for cbf in cbfs:
        maxVelocities = np.array([
            cbf.maxVelocityForCurrentDistance(distance)
            for distance in distances
        ])

        fig, ax = plt.subplots()

        ax.set_xlim(-0.1, totTime)

        fontSize = 25

        ax.set_ylabel(r'velocity (m/s)', fontsize=fontSize)
        ax.set_ylim(-0.05, DRONE_MAX_VELOCITY + 0.25)

        ax.set_xlabel(r'distance (m)', fontsize=fontSize)

        # function = r'$v_{safe\&fast} = min(v_{max}, \sqrt[+]{2 * | a_{max} | * (d_h - \epsilon)})$'
        # values = r'$v_{max} = $ ' + str(v_max) + r', $a_{max} = $ ' + str(a_max) + r', $\epsilon = $ ' + str(e)
        # ax.set_title(f'Fastest possible velocities based on distance to human that are still safe', fontsize=fontSize)
        sns.lineplot(distances, maxVelocities, marker="o", ax=ax)
        saveFig(
            fig,
            f"cbf - a_max={round(cbf.decceleration_max, 2)} | e={round(cbf.epsilon, 2)} | v_max={round(DRONE_MAX_VELOCITY, 2)}",
            30 / 25)

    # plt.show()


if __name__ == "__main__":
    # plotVelocities()
    h = HeuristicSafetyFunction()

    maxV = h.maxVelocityForCurrentDistance(2)
