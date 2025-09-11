ARG TARGET=cpu

FROM ghcr.io/aarhus-robotics/dyno:devel-stable AS cpu

FROM ghcr.io/aarhus-robotics/dyno:devel-stable-cuda AS cuda

FROM ${TARGET} AS common

ARG BUILD_TYPE=RelWithDebInfo
ARG DYNO_PATH="/usr/local/src/dyno"
ARG USER=robot

USER root

# ---------------------------------------------------------------------------- #
# If using the GitHub repository as origin for the source code, use this ADD
# command to invalidate the cache when new commits are available.
#
# ADD https://api.github.com/repos/aarhus-robotics/dyno/git/refs/heads/main version.json
# ---------------------------------------------------------------------------- #

RUN mkdir -p ${DYNO_PATH}/{cmake,data,include,src,tests}

COPY CMakeLists.txt poetry.lock pyproject.toml ${DYNO_PATH}
COPY cmake ${DYNO_PATH}/cmake
COPY data ${DYNO_PATH}/data
COPY include ${DYNO_PATH}/include
COPY src ${DYNO_PATH}/src
COPY tests ${DYNO_PATH}/tests

RUN mkdir -p ${DYNO_PATH}/build && \
    cd ${DYNO_PATH}/build && \
    cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE} && \
    cmake --build . -j$(nproc) --target install && \
    ldconfig
    
USER robot