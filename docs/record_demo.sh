#!/bin/bash
#  Copyright (c) 2026 KU Leuven, Belgium
#  Author: Santiago Iregui
#  GNU Lesser General Public License version 3
#
# Runs a crospi demo headless and records it with record_states.py (render it afterwards with render_gif.py):
# starts a crospi_node with the setup file, the recorder, and the BeTFSM script (from its own folder, since some
# scripts load their task list relative to the current directory) for at most <timeout> seconds (SIGINT afterwards,
# e.g. for scripts that loop forever). Then stops the recorder and the crospi_node.
#
# Usage (from a sourced workspace; all arguments are required):
#   docs/record_demo.sh <setup file> <BeTFSM script> <output.npz> <timeout [s]>
# Example:
#   docs/record_demo.sh '$[crospi_application_template]/applications/application_mujoco_examples_ur10e/ur10e_contour_following_circular.setup.json' \
#       <ws>/src/crospi_application_template/skill_specifications/libraries/skill_lib_example/circular_contour_following_example/circular_contour_following_example.py \
#       /tmp/circular.npz 240
#
# The visualization topics must be /mujoco_sim/model and /mujoco_sim/state (as in the example setup files).
# The logs are written next to the output: <output>.node.log, <output>.recorder.log, <output>.script.log.

set -u
if [ $# -ne 4 ]; then
    sed -n '/^# Usage/,/^# Example/p' "$0"
    exit 1
fi
SETUP=$1
SCRIPT=$(realpath "$2")
OUTPUT=$(realpath "$3")
TIMEOUT=$4
DOCS=$(cd "$(dirname "$0")" && pwd)

if pgrep -x crospi_node > /dev/null; then
    echo "A crospi_node is already running (pgrep -af crospi_node): stop it first." >&2
    exit 1
fi

ros2 run crospi_core crospi_node --ros-args -p config_file:="$SETUP" -p simulation:=true \
    > "$OUTPUT.node.log" 2>&1 &
NODE=$!
sleep 5
python3 "$DOCS/record_states.py" --model-topic /mujoco_sim/model --state-topic /mujoco_sim/state \
    --output "$OUTPUT" > "$OUTPUT.recorder.log" 2>&1 &
RECORDER=$!
sleep 2

(cd "$(dirname "$SCRIPT")" && timeout --signal=INT "$TIMEOUT" python3 "$SCRIPT" > "$OUTPUT.script.log" 2>&1)
echo "BeTFSM script exit code: $? (124: stopped by the timeout)"

kill -TERM $RECORDER; wait $RECORDER
kill -INT $NODE; sleep 2
pkill -x crospi_node   # ros2 run does not forward the signal to the node process
grep -h "outcome" "$OUTPUT.script.log"
cat "$OUTPUT.recorder.log" | grep "Saved\|No model"
