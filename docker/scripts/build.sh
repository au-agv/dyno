#!/bin/bash

set -e

PROGRESS=plain
VERSION=$(git rev-parse --short HEAD)
BUILD_TYPE=RelWithDebInfo

cd "$(dirname "${BASH_SOURCE[0]}")"./../

docker build --progress ${PROGRESS} \
             --build-arg TARGET=cpu \
             --build-arg BUILD_TYPE=${BUILD_TYPE} \
             --tag ghcr.io/aarhus-robotics/dyno:devel \
             --tag ghcr.io/aarhus-robotics/dyno:devel-${VERSION} \
             --tag ghcr.io/aarhus-robotics/dyno:devel-stable \
             --file docker/build/standalone/devel/Dockerfile \
             .

docker build --progress ${PROGRESS} \
             --build-arg TARGET=cpu \
             --build-arg BUILD_TYPE=${BUILD_TYPE} \
             --tag ghcr.io/aarhus-robotics/dyno \
             --tag ghcr.io/aarhus-robotics/dyno:runtime \
             --tag ghcr.io/aarhus-robotics/dyno:runtime-${VERSION} \
             --tag ghcr.io/aarhus-robotics/dyno:runtime-stable \
             --file docker/build/standalone/runtime/Dockerfile \
             .

if [ -f docker/build/standalone/devel/third_party/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh ]; then
    docker build --progress ${PROGRESS} \
                --build-arg TARGET=cuda \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/aarhus-robotics/dyno:devel-cuda \
                --tag ghcr.io/aarhus-robotics/dyno:devel-${VERSION}-cuda \
                --tag ghcr.io/aarhus-robotics/dyno:devel-stable-cuda \
                --file docker/build/standalone/devel/Dockerfile \
                .

    docker build --progress ${PROGRESS} \
                --build-arg TARGET=cuda \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/aarhus-robotics/dyno:cuda \
                --tag ghcr.io/aarhus-robotics/dyno:runtime-cuda \
                --tag ghcr.io/aarhus-robotics/dyno:runtime-${VERSION}-cuda \
                --tag ghcr.io/aarhus-robotics/dyno:runtime-stable-cuda \
                --file docker/build/standalone/runtime/Dockerfile \
                .

    docker build --progress ${PROGRESS} \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/aarhus-robotics/dyno:ros-devel \
                --tag ghcr.io/aarhus-robotics/dyno:ros-devel-${VERSION} \
                --tag ghcr.io/aarhus-robotics/dyno:ros-devel-stable \
                --file docker/build/ros/devel/Dockerfile \
                .

    docker build --progress ${PROGRESS} \
                --build-arg BUILD_TYPE=${BUILD_TYPE} \
                --tag ghcr.io/aarhus-robotics/dyno:ros \
                --tag ghcr.io/aarhus-robotics/dyno:ros-runtime \
                --tag ghcr.io/aarhus-robotics/dyno:ros-runtime-${VERSION} \
                --tag ghcr.io/aarhus-robotics/dyno:ros-runtime-stable \
                --file docker/build/ros/runtime/Dockerfile \
                .
fi