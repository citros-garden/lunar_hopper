#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/lunar_hopper/install/setup.bash 

exec "$@"
