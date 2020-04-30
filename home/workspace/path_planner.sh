#!/bin/bash
#Script to change directories and run scripts

cd /home/workspace/CarND-Path-Planning-Project

# Clean
echo Starting to clean ...
./clean.sh
echo Done!

# Build
echo Starting to build ...
./build.sh

cd build
if [[ -f path_planning ]]
then
    echo Done!
    cd /home/workspace/CarND-Path-Planning-Project
    # Run
    echo Starting to run ...
    ./run.sh
fi