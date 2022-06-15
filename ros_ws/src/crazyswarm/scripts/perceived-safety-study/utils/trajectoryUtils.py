import os

from SimpleTrajectory import SimpleTrajectory


def getLatestTrajectory():
    trajectoryFolder = "savedTrajectories"
    trajectoryFolders = [
        name for name in os.listdir(trajectoryFolder)
        if os.path.isdir(os.path.join(trajectoryFolder, name))
    ]
    # trajectoryFolders.sort(reverse=True)
    trajectoryFolders.sort(reverse=False)
    latestTrajectoryFolder = trajectoryFolders[0]

    pathToTrajectoryFolder = f"savedTrajectories/{latestTrajectoryFolder}"
    csvFilePath = f"{pathToTrajectoryFolder}/trajectoryData.csv"
    return SimpleTrajectory(csv=csvFilePath), pathToTrajectoryFolder
