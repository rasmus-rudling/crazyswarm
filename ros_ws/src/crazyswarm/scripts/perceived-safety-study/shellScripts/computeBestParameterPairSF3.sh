#! bin/bash

cd /home/rpl/Documents/rasmus/crazyswarm/ros_ws/src/crazyswarm/scripts/perceived-safety-study/mainStudy

echo 'Compute best parameter pair'
python3 main.py bppSF3 $1