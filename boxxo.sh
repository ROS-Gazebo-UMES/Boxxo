#!/bin/bash
catkin build
source devel/setup.bash

command=$1

case $command in
  "viz")
    roslaunch boxxo_core xacro_viz.launch
    ;;
  *)
    roslaunch boxxo_core boxxo.launch
    ;;
esac