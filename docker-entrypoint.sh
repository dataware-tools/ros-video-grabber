#!/usr/bin/env bash
#
# Entrypoint
#

source /opt/ros/*/setup.bash
source /opt/ros-video-grabber/devel/setup.bash

exec "$@"
