"""Takeoff-hover-land for one CF. Useful to validate hardware config."""
import sys
sys.path.append("..")
from pycrazyswarm import Crazyswarm


def main():
    crazyflies_yaml = str({
        'crazyflies': [{
            'channel': 100,
            'id': 7,
            'initialPosition': [0.0, 0.0, 0.0],
            'type': 'default'
        }]
    })
    swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
    timeHelper = swarm.timeHelper

    drone = swarm.allcfs.crazyflies[0]

    duration = 2.5

    drone.takeoff(targetHeight=1.0, duration=duration)
    timeHelper.sleep(duration + duration)

    drone.land(targetHeight=0.07, duration=duration)
    timeHelper.sleep(duration)


if __name__ == "__main__":
    main()