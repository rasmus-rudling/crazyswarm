import os
import sys

from matplotlib import pyplot as plt
import numpy as np
import rospy
sys.path.append("..")
from pycrazyswarm import Crazyswarm
from tf import TransformListener

def tf_callback(self, data):
  transform = data.transforms[0]
  pose = transform.transform


def main():
  crazyflies_yaml = str({'crazyflies': [{'channel': 100, 'id': 7, 'initialPosition': [0, 0, 0], 'type': 'default'}]})
  swarm = Crazyswarm(crazyflies_yaml=crazyflies_yaml)
  timeHelper = swarm.timeHelper
  drone = swarm.allcfs.crazyflies[0]

  # trajectoryData = np.loadtxt("hoverAndLandData.csv", delimiter=",", skiprows=1)
  trajectoryData = np.loadtxt("hoverForwardAndBackLand.csv", delimiter=",", skiprows=1)

  timestamps = np.array([row[0] for row in trajectoryData])
  x = np.array([row[1] for row in trajectoryData])
  y = np.array([row[2] for row in trajectoryData])
  z = np.array([row[3] for row in trajectoryData])
  yaw = np.array([row[4] for row in trajectoryData])

  timestamps -= timestamps[0]

  delta = 0.05  # In seconds, 0.05s = 20Hz
  t_d = np.arange(0, timestamps[-1], delta)
  num_states = t_d.shape[0]

  x_d = np.interp(t_d, timestamps, x)
  y_d = np.interp(t_d, timestamps, y)
  z_d = np.interp(t_d, timestamps, z)
  yaw_d = np.interp(t_d, timestamps, yaw)

  # Get init drone position
  tf = TransformListener()
  tf.waitForTransform("/world", "/cf7", rospy.Time(), rospy.Duration(4.0))

  t = tf.getLatestCommonTime("/world", "/cf7")
  init_position, quaternion = tf.lookupTransform('/world', '/cf7', t)

  init_position = np.array(init_position)

  error = init_position - [x_d[0], y_d[0], z_d[0]]
  record = list()

  for i in range(num_states):
    current_position = np.array([x_d[i], y_d[i], z_d[i]])
    position_command = current_position + error
    drone.cmdPosition(position_command, yaw_d[i])
    timeHelper.sleep(delta)

    t = tf.getLatestCommonTime("/world", "/cf7")
    record.append(tf.lookupTransform('/world', '/cf7', t)[0][2])

  plt.plot(t_d, z_d, "-r")
  plt.plot(t_d, record, "-g")
  
  plt.show()


if __name__ == "__main__":
  main()