import csv
import time
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
import numpy as np
from globalVariables import DRONE_MAX_VELOCITY
from planner.drone import DroneState
from planner.flyZone import MOCAP_FLY_ZONE, FlyZone
from planner.plotting import plotDroneState, plotFlyZone, plotlive

from plotUtils import saveFig


class SimpleTrajectory:
    def __init__(self,
                 timestamps=None,
                 x=None,
                 y=None,
                 z=None,
                 yaw=None,
                 csv=None,
                 poses=None,
                 z_height=None):

        if csv is not None:
            trajectoryData = np.loadtxt(csv, delimiter=",", skiprows=1)

            timestampsTemp = np.array([row[0] for row in trajectoryData])
            self.timestamps = timestampsTemp - timestampsTemp[0]
            self.x = np.array([row[1] for row in trajectoryData])
            self.y = np.array([row[2] for row in trajectoryData])

            # If trajectory is from motion planner, keep z
            self.z = np.array([
                row[3] if row[3] != -1997 else z_height
                for row in trajectoryData
            ])
            self.yaw = np.array([row[4] for row in trajectoryData])
        elif poses is not None:
            self.timestamps = timestamps

            x, y, z, yaw = [], [], [], []
            for pose in poses:
                x.append(pose.x)
                y.append(pose.y)

                # If trajectory is from motion planner, keep z
                z.append(pose.z) if pose.z != -1997 else z_height
                yaw.append(pose.yaw)

            self.x = np.array(x)
            self.y = np.array(y)
            self.z = np.array(z)
            self.yaw = np.array(yaw)
        else:
            if len(timestamps) == 0:
                print("No values in trajectory")
                exit()

            self.timestamps = timestamps - timestamps[0]
            self.x = x
            self.y = y
            self.z = z if z[0] != -1997 else np.full(
                len(x),
                z_height)  # If trajectory is from motion planner, keep z
            self.yaw = yaw

    def saveTrajectoryToCsv(self, fileName):
        with open(fileName, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            header = ["timestamp", "x", "y", "z", "yaw"]

            writer.writerow(header)

            numDroneStates = len(self.timestamps)
            for i in range(numDroneStates):
                row = [
                    self.timestamps[i], self.x[i] / 100, self.y[i] / 100,
                    self.z[i] / 100, self.yaw[i]
                ]

                writer.writerow(row)

    def interpolateTrajectory(self, delta: float):
        """

    Args:
        delta (float): Distance between each data point in the interpolated trajectory.

    Returns:
        SimpleTrajectory: Interpolated trajectory.
    """
        t_d = np.arange(0, self.timestamps[-1], delta)

        x_d = np.interp(t_d, self.timestamps, self.x)
        y_d = np.interp(t_d, self.timestamps, self.y)
        z_d = np.interp(t_d, self.timestamps, self.z)
        yaw_d = np.interp(t_d, self.timestamps, self.yaw)

        interpolatedTrajectory = SimpleTrajectory(t_d, x_d, y_d, z_d, yaw_d)
        return interpolatedTrajectory

    def getVelocites(self):
        currentPositions = [
            np.array([self.x[i], self.y[i], self.z[i]])
            for i in range(len(self.timestamps))
        ]
        goalPositions = [
            np.array([self.x[i + 1], self.y[i + 1], self.z[i + 1]])
            for i in range(len(self.timestamps) - 1)
        ]
        goalPositions.append(np.array([self.x[-1], self.y[-1], self.z[-1]]))
        time = self.timestamps[1] - self.timestamps[0]

        currentPositions = np.array(currentPositions)
        goalPositions = np.array(goalPositions)

        distanceToGoalPosition = np.array([
            np.linalg.norm(currentPositions[i] - goalPositions[i])
            for i in range(len(currentPositions))
        ])
        velocities = distanceToGoalPosition / time

        return velocities

    def plotTrajectory(self,
                       otherTrajectory: "SimpleTrajectory" = None,
                       fileName=None):
        fig, axs = plt.subplots(4, sharex=True)

        if len(self.timestamps) == 0:
            print("No values in trajectory")
            return

        for ax in axs:
            ax.set_xlabel("time (s)")

        plannedVelocities = self.getVelocites()
        axs[0].plot(self.timestamps,
                    plannedVelocities,
                    "-g",
                    label="Planned trajectory")

        axs[1].plot(self.timestamps, self.x, "-g")
        axs[2].plot(self.timestamps, self.y, "-g")
        axs[3].plot(self.timestamps, self.z, "-g")

        axs[0].set_ylabel("velocity (m/s)")
        axs[1].set_ylabel("x (m)")
        axs[2].set_ylabel("y (m)")
        axs[3].set_ylabel("z (m)")

        axs[0].set_ylim(-DRONE_MAX_VELOCITY - 0.1, DRONE_MAX_VELOCITY + 0.1)
        axs[3].set_ylim(0, 2.5)

        if otherTrajectory is not None and len(otherTrajectory.timestamps) > 0:
            executedVelocities = otherTrajectory.getVelocites()
            axs[0].plot(otherTrajectory.timestamps,
                        executedVelocities,
                        "-r",
                        label="Executed trajectory")

            axs[1].plot(otherTrajectory.timestamps, otherTrajectory.x, "-r")
            axs[2].plot(otherTrajectory.timestamps, otherTrajectory.y, "-r")
            axs[3].plot(otherTrajectory.timestamps, otherTrajectory.z, "-r")

        axs[0].legend()

        if fileName is not None:
            saveFig(fig, fileName)

        # plt.show()
        # plt.pause(300000000)

    def slowTrajectory(self, factor: 2 | 3 | 4, delta=0.1):
        """Slow down the trajectory by a factor of 2, 3, or 4."""
        newFinalTimestep = self.timestamps[-1] * factor
        t_d = np.arange(0, newFinalTimestep, delta)

        x_d = np.interp(t_d, self.timestamps, self.x)
        y_d = np.interp(t_d, self.timestamps, self.y)
        z_d = np.linspace(self.z[0], self.z[-1], len(t_d))
        yaw_d = np.interp(t_d, self.timestamps, self.yaw)

        self.timestamps = t_d
        self.x = x_d
        self.y = y_d
        self.z = z_d
        self.yaw = yaw_d

    def slowTrajectoryOld(self, factor: 2 | 3 | 4):
        originalNumTimestamps = len(self.timestamps)

        newX = []
        newY = []
        newZ = []
        newYaw = []

        counter = 0
        for i in range(originalNumTimestamps):
            for _ in range(factor):
                counter += 1

                newX.append(self.x[i])
                newY.append(self.y[i])
                newZ.append(self.z[i])
                newYaw.append(self.yaw[i])

        self.x = np.array(newX)
        self.y = np.array(newY)
        self.z = np.array(newZ)
        self.yaw = np.array(newYaw)

        # tDelta = self.timestamps[1] - self.timestamps[0]
        newFinalTimestep = self.timestamps[-1] * factor
        newNumTimesteps = len(newX)

        self.timestamps = np.linspace(0, newFinalTimestep, newNumTimesteps)

    def visualize(self):
        fig, ax = plt.subplots()

        for i, t_curr in enumerate(self.timestamps):
            self.plotDronePose(ax)
            x, y, yaw = self.x[i], self.y[i], self.yaw[i]

            droneState = DroneState(parent=None, x=x, y=y, yaw=yaw)
            plotDroneState(ax, droneState)

            plt.show()

            if i < len(self.timestamps) - 1:
                t_next = self.timestamps[i + 1]
                t_sleep = t_next - t_curr
                plt.pause(t_sleep)
            else:
                plt.pause(2)
                plt.close(fig)

    @plotlive
    def plotDronePose(self, ax: "Axes"):
        plotFlyZone(ax, MOCAP_FLY_ZONE)

    def __str__(self):
        return f"-Trajectory-\ntimestamps={self.timestamps}\nx={self.x}\ny={self.y}\nz={self.z}\nyaw={np.round(np.rad2deg(self.yaw))}\n"
