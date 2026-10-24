#!/bin/bash
# spawn_robot.sh - Wait for Gazebo, delete stale entity, then spawn robot
# Usage: spawn_robot.sh <model_name> <urdf_param> <x> <y> <z> <Y>
set -e
model_name="$1"
urdf_param="$2"
x="$3"
y="$4"
z="$5"
Y="$6"

# Wait for Gazebo spawn service (up to 30s)
rosservice --timeout 30 wait /gazebo/spawn_urdf_model 2>/dev/null || true

# Delete stale entity (ignore errors if it doesn't exist)
rosrun gazebo_ros delete_entity -model "$model_name" 2>/dev/null || true
sleep 1

# Spawn the model
exec rosrun gazebo_ros spawn_model -urdf -param "$urdf_param" \
    -model "$model_name" -x "$x" -y "$y" -z "$z" -Y "$Y"
