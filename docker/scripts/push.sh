#!/bin/bash

set -e

VERSION=$(git rev-parse --short HEAD)

docker push ghcr.io/aarhus-robotics/dyno:devel
docker push ghcr.io/aarhus-robotics/dyno:devel-${VERSION}
docker push ghcr.io/aarhus-robotics/dyno:devel-stable

docker push ghcr.io/aarhus-robotics/dyno
docker push ghcr.io/aarhus-robotics/dyno:runtime
docker push ghcr.io/aarhus-robotics/dyno:runtime-${VERSION}
docker push ghcr.io/aarhus-robotics/dyno:runtime-stable

docker push ghcr.io/aarhus-robotics/dyno:devel-cuda
docker push ghcr.io/aarhus-robotics/dyno:devel-${VERSION}-cuda
docker push ghcr.io/aarhus-robotics/dyno:devel-stable-cuda

docker push ghcr.io/aarhus-robotics/dyno:cuda
docker push ghcr.io/aarhus-robotics/dyno:runtime-cuda
docker push ghcr.io/aarhus-robotics/dyno:runtime-${VERSION}-cuda
docker push ghcr.io/aarhus-robotics/dyno:runtime-stable-cuda

docker push ghcr.io/aarhus-robotics/dyno:ros
docker push ghcr.io/aarhus-robotics/dyno:ros-${VERSION}
docker push ghcr.io/aarhus-robotics/dyno:ros-stable