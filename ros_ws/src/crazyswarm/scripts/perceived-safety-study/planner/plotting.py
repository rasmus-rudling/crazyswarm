if __name__ == "__main__":
    from planner import Planner

import sys

sys.path.append(
    '/Users/rr/Documents/thesis/degree-thesis/ros_ws/src/crazyswarm/scripts/perceived-safety-study/utils'
)

import functools
import time
from typing import List

from matplotlib.axes import Axes
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from planner.drone import DroneState
from planner.helperFunctions import cmToInches, getDirectionVectorFromAngleAndLength
from planner.flyZone import FlyZone
from matplotlib.patches import Circle, FancyArrowPatch, ConnectionPatch
from colour import Color
import numpy as np
from position import Position
from utils.globalVariables import PATH_TO_ROOT

np.random.seed(0)
import seaborn as sns

sns.set_theme()
from planner.obstacle import Obstacle

MAX_Z_ORDER = 999999999999999999999999999999999999999999999999


def plotFlyZone(ax: Axes, flyZone: "FlyZone"):
    """Plot the fly zone and its' padding if it have that.

  Args:
      ax (Axes): The axes where the fly zone should be plotted.
      flyZone (FlyZone): The flyzone to be plotted.
  """
    MARGIN = 0.1

    ax.set_aspect('equal')

    xLimMin = flyZone.minPos.x - MARGIN
    xLimMax = flyZone.maxPos.x + MARGIN
    ax.set_xlim(xLimMin, xLimMax)

    yLimMin = flyZone.minPos.y - MARGIN
    yLimMax = flyZone.maxPos.y + MARGIN
    ax.set_ylim(yLimMin, yLimMax)

    zoneWidth = flyZone.maxPos.x - flyZone.minPos.x
    zoneHeight = flyZone.maxPos.y - flyZone.minPos.y

    flyZonePatch = patches.Rectangle((flyZone.minPos.x, flyZone.minPos.y),
                                     zoneWidth,
                                     zoneHeight,
                                     edgecolor='r',
                                     facecolor='none',
                                     zorder=60)
    ax.add_patch(flyZonePatch)

    # Plot padded fly zone
    if flyZone.flyZoneWithPadding is not None:
        paddingZoneWidth = flyZone.flyZoneWithPadding.maxPos.x - flyZone.flyZoneWithPadding.minPos.x
        paddingZoneHeight = flyZone.flyZoneWithPadding.maxPos.y - flyZone.flyZoneWithPadding.minPos.y

        flyZoneWithPaddingPatch = patches.Rectangle(
            (flyZone.flyZoneWithPadding.minPos.x,
             flyZone.flyZoneWithPadding.minPos.y),
            paddingZoneWidth,
            paddingZoneHeight,
            edgecolor='b',
            facecolor='none',
            zorder=60)
        ax.add_patch(flyZoneWithPaddingPatch)


def plotObstacles(ax: Axes,
                  obstacles: List["Obstacle"],
                  color: str = "red",
                  text: str = "Obstacle",
                  zorder: int = 50):
    """Plot all obstacles.

  Args:
      ax (Axes): The axes where the obstacles should be plotted.
      obstacles (List[Obstacle]): The obstacles to be plotted.
      color (str, optional): Background color of each obstacle. Defaults to "red".
      text (str, optional): What should be printed on all the obstacles. Defaults to "Obstacle".
      zorder (int, optional): The z-order of the obstacle patches. Defaults to 50.
  """
    for obstacle in obstacles:
        obstacleCenter = (obstacle.x, obstacle.y)
        patch = Circle(obstacleCenter,
                       obstacle.radius,
                       color=color,
                       zorder=zorder)
        ax.add_artist(patch)

        ax.annotate(text,
                    obstacleCenter,
                    color='w',
                    weight='bold',
                    fontsize=9,
                    ha='center',
                    va='center',
                    zorder=zorder + 1)


def plotDroneState(ax: Axes,
                   droneState: "DroneState",
                   color: str = "#232323",
                   text: str = "D") -> None:
    """Plots the most recent / current drone state including its' yaw.

  Args:
      ax (Axes): The axes where the drone should be plotted.
      droneState (DroneState): The most recent drone state.
      color (str, optional): The background color that will be used to visualize the drone state. Defaults to "#232323".
      text (str, optional): The text that shows on the drone state. Defaults to "D".
  """
    droneRadius = droneState.DRONE_INFO.radius
    droneCenter = (droneState.x, droneState.y)

    dronePatch = Circle(droneCenter,
                        droneRadius,
                        color=color,
                        zorder=MAX_Z_ORDER)

    xMovement, yMovement = getDirectionVectorFromAngleAndLength(
        angle=droneState.yaw, length=droneRadius + 0.11)

    xHead, yHead = droneState.x + xMovement, droneState.y + yMovement

    try:
        arrowPatch = FancyArrowPatch(droneCenter, (xHead, yHead),
                                     mutation_scale=20,
                                     color=color,
                                     zorder=MAX_Z_ORDER - 1)

        ax.add_patch(arrowPatch)
        ax.add_patch(dronePatch)
        ax.annotate(text,
                    droneCenter,
                    color='w',
                    weight='bold',
                    fontsize=9,
                    ha='center',
                    va='center',
                    zorder=MAX_Z_ORDER + 1)
    except:
        print("Couldn't print drone direction arrow")


def savePlot(fig: Figure,
             folderNameForAnimation,
             animationName,
             saveInRoot=False) -> None:
    ratio = 30 / 24
    width_in_cm = 25
    w = cmToInches(width_in_cm)
    h = cmToInches(width_in_cm / ratio)
    fig.set_size_inches(w, h)
    # fig.savefig(f"animationFrames/{folderNameForAnimation}/{animationNum}.png", format='png', dpi=144)

    if saveInRoot:
        try:
            fig.savefig(
                f"{PATH_TO_ROOT}/savedTrajectories/comparisons/{folderNameForAnimation}/{animationName}.png",
                format='png',
                dpi=120)
        except:
            try:
                fig.savefig(
                    f"{PATH_TO_ROOT}/savedTrajectories/{folderNameForAnimation}/{animationName}.png",
                    format='png',
                    dpi=120)
            except:
                try:
                    fig.savefig(
                        f"{PATH_TO_ROOT}/preStudy/{folderNameForAnimation}/{animationName}.png",
                        format='png',
                        dpi=120)
                except:
                    try:
                        fig.savefig(
                            f"{PATH_TO_ROOT}/mainStudy/participants/{folderNameForAnimation}/{animationName}.png",
                            format='png',
                            dpi=120)
                    except:
                        fig.savefig(
                            f"{PATH_TO_ROOT}/{folderNameForAnimation}/trajectory.png",
                            format='png',
                            dpi=120)

    else:
        try:
            fig.savefig(
                f"savedTrajectories/{folderNameForAnimation}/animationFrames/{animationName}.png",
                format='png',
                dpi=120)
        except:
            fig.savefig(
                f"{PATH_TO_ROOT}/{folderNameForAnimation}/animationFrames/{animationName}.png",
                format='png',
                dpi=120)


def plotPathToCurrentDroneStateLive(droneState: "DroneState",
                                    planner: "Planner",
                                    ax: Axes,
                                    fig: Figure,
                                    isFinalDroneState: bool,
                                    fileName=None) -> bool:
    startX, startY = round(planner.currentStartState.x,
                           2), round(planner.currentStartState.y, 2)
    startPos = f"p(S)=({startX}, {startY})"

    goalX, goalY = round(planner.currentGoalState.x,
                         2), round(planner.currentGoalState.y, 2)
    goalPos = f"p(G)=({goalX}, {goalY})"

    currentX, currentY = round(droneState.x, 2), round(droneState.y, 2)
    currentPos = f"p(D)=({currentX}, {currentY})"

    currentVel = f"v(D)={round(droneState.velocity, 2)}m/s"
    movementCost = f"d_goal={round(droneState.parent.selectedMovement.newDistanceToGoal, 2)}m" if droneState.parent is not None else "d_goal=Ø"

    computationTime = f"Computation time = {round(planner.computationTime, 2)}s"

    distanceToHuman = f"d_human={round(droneState.parent.selectedMovement.newDistanceToHuman, 2)}m" if droneState.parent is not None else "d_human=Ø"

    plt.title(
        f"{computationTime}\n\n{startPos} ➙ {goalPos}\n\ni={planner.iterations} | D={droneState.depth} | {currentPos} | {currentVel} | {movementCost} | {distanceToHuman}",
        fontsize=12)
    plotFlyZone(ax, flyZone=planner.flyZone)
    plotObstacles(ax, obstacles=planner.obstacles)
    plotObstacles(ax,
                  obstacles=[planner.HUMAN],
                  color="blue",
                  text="Human",
                  zorder=100)
    plotPosition(ax,
                 position=planner.currentGoalState,
                 radius=planner.currentGoalState.radius)
    plotPathToCurrentState(ax,
                           droneState,
                           isFinalDroneState=isFinalDroneState,
                           planner=planner)
    # uniform_data = np.random.rand(10, 12)
    # ax = sns.heatmap(uniform_data)

    if planner.sf.name == "heuristic":
        h = planner.HUMAN

        dronePatch = Circle((h.x, h.y), 0.45, color="red", zorder=50)
        ax.add_patch(dronePatch)
        ax.annotate("0m/s", (h.x, h.y + 0.35),
                    color='w',
                    weight='bold',
                    ha='center',
                    va='center',
                    fontsize=9,
                    zorder=51)

        dronePatch = Circle(((h.x, h.y)), 1.2, color="orange", zorder=40)
        ax.add_patch(dronePatch)
        ax.annotate("0.75m/s", (h.x, h.y + 1.1),
                    color='w',
                    weight='bold',
                    ha='center',
                    va='center',
                    fontsize=9,
                    zorder=41)

        dronePatch = Circle((h.x, h.y), 3.6, color="pink", zorder=30)
        ax.add_patch(dronePatch)
        ax.annotate("1.5m/s", (h.x, h.y + 1.5),
                    color='w',
                    weight='bold',
                    ha='center',
                    va='center',
                    fontsize=9,
                    zorder=31)

        dronePatch = Circle((h.x, h.y), 7.6, color="purple", zorder=20)
        ax.add_patch(dronePatch)
        ax.annotate("1.5m/s", (h.x, h.y),
                    color='w',
                    weight='bold',
                    ha='center',
                    va='center',
                    fontsize=9,
                    zorder=21)

    if planner.saveAnimation:
        animationName = planner.currentAnimationFrame if fileName is None else fileName

        saveInRoot = fileName is not None

        savePlot(fig,
                 folderNameForAnimation=planner.animationKey,
                 animationName=animationName,
                 saveInRoot=saveInRoot)

    plt.pause(0.0000001)
    plt.show()


def plotPathFromPreviousStateToCurrentState(ax: Axes,
                                            currentDroneState: "DroneState",
                                            previousDroneState: "DroneState",
                                            planner) -> bool:
    numPossibleAccelerations = len(planner.possibleAccelerations)

    green = Color("#16a34a")
    red = Color("#dc2626")
    colors = list(green.range_to(red, numPossibleAccelerations))

    acceleration = previousDroneState.selectedMovement.acceleration
    actionIdx = planner.possibleAccelerations.index(acceleration)

    color = colors[actionIdx].hex

    if planner.possibleAccelerations[actionIdx] == 0.0:
        color = "#c4c4c4"

    plotPosition(ax,
                 previousDroneState,
                 0.1,
                 color=color,
                 text=f"{previousDroneState.depth}",
                 zorder=previousDroneState.depth * 2 + 100)

    connectionPatch = ConnectionPatch(xyA=(previousDroneState.x,
                                           previousDroneState.y),
                                      coordsA="data",
                                      xyB=(currentDroneState.x,
                                           currentDroneState.y),
                                      coordsB="data",
                                      arrowstyle="-",
                                      color=color,
                                      zorder=0)

    ax.add_patch(connectionPatch)


def plotPathToCurrentState(ax: Axes, droneState: "DroneState",
                           isFinalDroneState: bool, planner) -> bool:
    if isFinalDroneState:
        plotDroneState(ax, droneState, "#E3BE00")
    else:
        plotDroneState(ax, droneState)

    currentDroneState = droneState

    while currentDroneState.parent is not None:
        plotPathFromPreviousStateToCurrentState(ax, currentDroneState,
                                                currentDroneState.parent,
                                                planner)

        currentDroneState = currentDroneState.parent


def plotPathToCurrentDroneState(planner: "Planner",
                                droneState: "DroneState") -> bool:
    ax = plt.subplots()[1]

    dt = 0.001

    while True:
        planner.plotStateLive(droneState=droneState, ax=ax, dt=dt)

        action = input("Action: ")

        if action == "b":
            break
        elif action == "q":
            exit(0)

        time.sleep(dt)


def plotPosition(ax: Axes,
                 position: "Position",
                 radius: float,
                 color: str = "#16a34a",
                 text: str = "Goal",
                 zorder: int = 0):
    posCenter = (position.x, position.y)

    edgecolor = color

    if text == "Goal":
        zorder = MAX_Z_ORDER - 5
        edgecolor = "#15803d"

    dronePatch = Circle(posCenter,
                        radius,
                        facecolor=color,
                        edgecolor=edgecolor,
                        zorder=zorder)
    ax.add_patch(dronePatch)
    ax.annotate(text,
                posCenter,
                color='w',
                weight='bold',
                fontsize=9,
                ha='center',
                va='center',
                zorder=zorder + 1)


def plotlive(func):
    plt.ion()

    @functools.wraps(func)
    def new_func(*args, **kwargs):

        # Clear all axes in the current figure.
        axes = plt.gcf().get_axes()
        for axis in axes:
            axis.cla()

        # Call func to plot something
        result = func(*args, **kwargs)

        # Draw the plot
        plt.draw()

        return result

    return new_func
