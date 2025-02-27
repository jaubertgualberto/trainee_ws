#!/bin/bash

# handle errors
set -e

# setup workspace
source /opt/ros/humble/setup.bash
cd atwork_ws

# runs any provided arguments
exec $@