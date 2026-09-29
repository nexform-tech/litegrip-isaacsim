# Dev container image for the litegrip-isaacsim repository.
#
# Built on the official NVIDIA Isaac Sim container (standalone layout at
# /isaac-sim, matching what run_gripper.sh expects), plus the small set of OS
# tools the repository's workflows need: SocketCAN tooling for the real-gripper
# bridge, git for the dev container lifecycle, and an editor/pager for shell
# work.
ARG ISAAC_SIM_VERSION=4.5.0
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION}

# The NGC image's entrypoint launches Isaac Sim itself. A dev container must
# run the VS Code server lifecycle instead, so the entrypoint is cleared and
# the command becomes a plain shell.
ENTRYPOINT []
CMD ["/bin/bash"]

RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      can-utils \
      git \
      iproute2 \
      less \
      vim-tiny \
 && rm -rf /var/lib/apt/lists/*
