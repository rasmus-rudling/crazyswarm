import sys

sys.path.append("./latticeStuff")

from latticeStuff.latticePlanner import *
from followTrajectory import executeCustomTrajectory
from pycrazyswarm import Crazyswarm
sys.path.append("..")

MOCAP_ROOM_HEIGHT = 2.57
MOCAP_ROOM_LENGTH = 4.5
MOCAP_ROOM_WIDTH = 3.55

HUMAN_RADIUS = 0.5
DRONE_RADIUS = 0.2
MIN_ALLOWED_DISTANCE_TO_HUMAN = 0.45


class State:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


class Drone:
    def __init__(self, init_position):
        self.position = init_position


def main():
    DRONE_HEIGHT = 2.0
    state1 = State(1.75, -1.65, DRONE_HEIGHT)
    state2 = State(-1.75, 1.35, DRONE_HEIGHT)
    state3 = State(-2.2, -2.1, DRONE_HEIGHT)

    states = [state1, state2, state3]

    current_position_idx = 0
    current_destination_idx = 1

    numTrajectories = 4

    currentState = states[0]
    destinationState = states[1]

    haveInitialized = False

    for trajectoryIdx in range(numTrajectories):
        obstacleRadius = (HUMAN_RADIUS + DRONE_RADIUS +
                          MIN_ALLOWED_DISTANCE_TO_HUMAN)

        latticePaths, tentacleLength = getLatticePaths()

        obstacle = Obstacle(0, 0, obstacleRadius)

        # print(f"Current (x,y)=({currentState.x}, {currentState.y})")

        trajectoryPlanner = TrajectoryPlanner(
            latticePaths=latticePaths,
            obstacle=obstacle,
            dt=0.1,
            startYaw=currentState.yaw,
            startX=currentState.x,
            startY=currentState.y,
            goalX=destinationState.x,
            goalY=destinationState.y,
        )

        trajectoryToGoal = trajectoryPlanner.findPathToGoal()

        trajectoryToGoal.saveTrajectoryToCsv("csvs/PathPlanningTrajectory.csv")
        trajectoryToFollow = TrajectoryUtils("csvs/PathPlanningTrajectory.csv",
                                             tentacleLength=tentacleLength)

        if not haveInitialized:
            startX = trajectoryToFollow.positions[0][0]
            startY = trajectoryToFollow.positions[0][1]
            startYaw = trajectoryToFollow.yaws[0]

            obstacleRadius = obstacle.radius - DRONE_RADIUS

            crazyflies_yaml = str({
                'crazyflies': [{
                    'channel': 100,
                    'id': 5,
                    'initialPosition': [startX, startY, startYaw],
                    'type': 'default'
                }]
            })
            swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml,
                               obstacleRadius=obstacleRadius)
            timeHelper = swarm.timeHelper
            drone = swarm.allcfs.crazyflies[0]
            plt.gca().view_init(elev=-90, azim=-90)

            haveInitialized = True

            drone.takeoff(targetHeight=DRONE_HEIGHT, duration=2)
            timeHelper.sleep(2)

        rate = 15  # In Hz
        trajectoryLogger = TrajectoryUtils()

        executeCustomTrajectory(timeHelper, drone, rate, trajectoryLogger,
                                trajectoryToFollow, DRONE_HEIGHT)
        trajectoryLogger.saveTrajectoryToCsv(
            f'csvs/loggedLatticeTrajectory{trajectoryIdx}.csv')

        print("Follower done!")

        current_destination_idx = (current_destination_idx + 1) % len(states)
        current_position_idx = (current_position_idx + 1) % len(states)

        currentState = states[current_position_idx]
        # print(f"old yaw = {round(np.rad2deg(currentState.yaw) % 360, 2)}°")
        # print(f"new yaw = {round(np.rad2deg(trajectoryToGoal.yaws[-1]) % 360, 2)}°")
        currentState.yaw = trajectoryToGoal.yaws[-1]
        destinationState = states[current_destination_idx]

        # timeHelper.sleep(2)

    drone.notifySetpointsStop()
    drone.land(targetHeight=0.03, duration=0.5)
    timeHelper.sleep(0.5)
    plt.close()


if __name__ == "__main__":
    main()
