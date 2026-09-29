#!/bin/bash
# Post-create hook for the litegrip-isaacsim dev container: verify the bundled
# Isaac Sim install, then run the repository's hardware-free unit tests as a
# smoke check. Runs once after the container is created; the working directory
# is the workspace root.
set -euo pipefail

ISAAC_SIM="${ISAAC_SIM_PATH:-/isaac-sim}"

echo "[devcontainer] checking the Isaac Sim install at $ISAAC_SIM"
if [[ ! -d "$ISAAC_SIM" ]]; then
  echo "[devcontainer] ERROR: $ISAAC_SIM is missing" >&2
  exit 1
fi

missing=""
for entry in setup_python_env.sh setup_ros_env.sh kit/python/bin/python3 exts apps; do
  if [[ ! -e "$ISAAC_SIM/$entry" ]]; then
    missing="$missing $entry"
  fi
done
if [[ -n "$missing" ]]; then
  echo "[devcontainer] WARNING: missing from the install:$missing" >&2
  echo "[devcontainer] run_gripper.sh expects the standalone layout; see .devcontainer/README.md" >&2
fi

echo "[devcontainer] running the hardware-free unit tests"
python3 -m unittest discover -s tests -v

echo "[devcontainer] ready: run ./run_gripper.sh to start the simulation"
