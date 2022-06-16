import numpy as np

from planner.cbFunctions import CBF

NUM_VALUES = 50

epsilonRange = np.linspace(CBF.EPSILON_MIN, CBF.EPSILON_MAX, NUM_VALUES)
decelerationMaxRange = np.linspace(CBF.DECELERATION_MAX_MIN,
                                   CBF.DECELERATION_MAX_MAX, NUM_VALUES)

predictions = np.zeros((epsilonRange.size, decelerationMaxRange.size))
standard_deviations = np.zeros((epsilonRange.size, decelerationMaxRange.size))

nextEpsilonToCheck = round(np.random.choice(epsilonRange), 2)
nextdecelerationMaxToCheck = round(np.random.choice(decelerationMaxRange), 2)

print(nextEpsilonToCheck)
print(nextdecelerationMaxToCheck)