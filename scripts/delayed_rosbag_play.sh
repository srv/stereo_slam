#!/bin/bash

# @file delayed_rosbag_play.sh
# @brief Script to delay the playback of a bagfile and thus 
#        ensure that the rest of the nodes have finished initialising.
# @package stereo_slam
# @project stereo_slam
#
# @author
#   Bo Miquel Nordfeldt-Fiol (2024–2025)
#
# @license BSD-4-Clause
#   This file is part of the stereo_slam project and is released under
#   the BSD 4-Clause License. See the LICENSE file for details.


# Default values.
DELAY=0.5
ROSBAG_ARGS=""

# Parser.
while [[ $# -gt 0 ]]; do
    case "$1" in
        --delay)
        DELAY="$2"
        shift 2
        ;;
        --rosbag-args)
        ROSBAG_ARGS="$2"
        shift 2
        ;;
        --)
        shift
        break
        ;;
        *)
        break
        ;;
    esac
done

# Sanity check.
if [[ -z "$ROSBAG_ARGS" ]]; then
    echo "ERROR: You must provide --rosbag-args \'<arguments for rosbag play>\'"
    exit 1
fi

# Delay rosbag play.
sleep "$DELAY"

# Play bagfile.
rosbag play $ROSBAG_ARGS