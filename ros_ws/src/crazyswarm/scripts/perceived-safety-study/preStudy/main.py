

import sys
from GaussianProcess import GaussianProcess

def createSafetyFunction(gp: "GaussianProcess"):
    pass


def main():
  gp = GaussianProcess()
  
  if len(sys.argv) <= 1:
    gp.startProcess()
  elif sys.argv[1] == "plot":
    gp.plotCurrentPredictionAs3d()


if __name__ == "__main__":
    main()