#!/bin/bash

# current directory
cwd=$(pwd)

# install brc_msgs interfaces
echo 'installing interfaces...'
cd ../brc_msgs
colcon build
source install/setup.bash

# build packages
echo ' '
echo 'installing packages...'
cd "${cwd}"
colcon build --base-paths src
source install/setup.bash