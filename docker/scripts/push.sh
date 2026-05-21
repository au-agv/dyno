#!/bin/bash

set -e

VERSION=$(git rev-parse --short HEAD)

docker push ghcr.io/au-agv/dyno:devel
docker push ghcr.io/au-agv/dyno:devel-${VERSION}
docker push ghcr.io/au-agv/dyno:devel-stable

docker push ghcr.io/au-agv/dyno
docker push ghcr.io/au-agv/dyno:runtime
docker push ghcr.io/au-agv/dyno:runtime-${VERSION}
docker push ghcr.io/au-agv/dyno:runtime-stable

# Disabled due to the NVIDIA OptiX dependency
#docker push ghcr.io/au-agv/dyno:devel-cuda
#docker push ghcr.io/au-agv/dyno:devel-${VERSION}-cuda
#docker push ghcr.io/au-agv/dyno:devel-stable-cuda

# Disabled due to the NVIDIA OptiX dependency
#docker push ghcr.io/au-agv/dyno:cuda
#docker push ghcr.io/au-agv/dyno:runtime-cuda
#docker push ghcr.io/au-agv/dyno:runtime-${VERSION}-cuda
#docker push ghcr.io/au-agv/dyno:runtime-stable-cuda

# Disabled due to the NVIDIA OptiX dependency
#docker push ghcr.io/au-agv/dyno:ros
#docker push ghcr.io/au-agv/dyno:ros-${VERSION}
#docker push ghcr.io/au-agv/dyno:ros-stable