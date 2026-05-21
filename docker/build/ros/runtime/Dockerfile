FROM ghcr.io/au-agv/dyno:ros-devel

ARG ROS_DISTRIBUTION=humble
ARG BUILD_TYPE=RelWithDebInfo
ARG USER=robot
ARG DYNO_PATH="/home/${USER}/dyno"

USER ${USER}

RUN mkdir -p ${DYNO_PATH}/{cmake,data,include,src,tests}

COPY --chown=${USER}:${USER} CMakeLists.txt pyproject.toml README.md ${DYNO_PATH}
COPY --chown=${USER}:${USER} cmake ${DYNO_PATH}/cmake
COPY --chown=${USER}:${USER} data ${DYNO_PATH}/data
COPY --chown=${USER}:${USER} include ${DYNO_PATH}/include
COPY --chown=${USER}:${USER} pydyno ${DYNO_PATH}/pydyno
COPY --chown=${USER}:${USER} ros /home/${USER}/ROS/src/au-agv/dyno/
COPY --chown=${USER}:${USER} src ${DYNO_PATH}/src
COPY --chown=${USER}:${USER} tests ${DYNO_PATH}/tests

RUN mkdir -p ${DYNO_PATH}/build && \
    cd ${DYNO_PATH}/build && \
    cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE} && \
    cmake --build . -j$(nproc)

USER root

RUN cd ${DYNO_PATH}/build && \
    cmake --build . --target install && \
    ldconfig

USER ${USER}

RUN cd /home/${USER}/ROS && \
    source /opt/ros/${ROS_DISTRIBUTION}/setup.bash && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

ENV VIRTUAL_ENV=/home/${USER}/.venv
ENV PATH=/home/${USER}/.venv/bin:$PATH

RUN cd ${DYNO_PATH} && \
    python3 -m venv /home/${USER}/.venv && \
    poetry lock && poetry install

COPY --chown=${USER}:${USER} examples ${DYNO_PATH}/examples
COPY --chmod='+x' docker/build/ros/runtime/entrypoint.py /home/${USER}/entrypoint.py

CMD ${HOME}/entrypoint.py

ENV DOCKER_ROS_ACTION=run
ENV DOCKER_ROS_PACKAGE=dyno_simulation
ENV DOCKER_ROS_EXECUTABLE=dyno_simulation_native_simulator_interface_node

###############################################################################
# Add container labels                                                        #
###############################################################################

LABEL org.opencontainers.image.authors="Dario Sirangelo (dev@dariosirangelo.me)"
LABEL org.opencontainers.image.url=https://github.com/au-agv/dyno
LABEL org.opencontainers.image.documentation=https://au-agv.github.io/dyno
LABEL org.opencontainers.image.source=https://github.com/au-agv/dyno/tree/main/docker/build/ros/runtime
LABEL org.opencontainers.image.version=main
LABEL org.opencontainers.image.licenses=MIT
LABEL org.opencontainers.image.title="DYNO (ros-runtime)"
LABEL org.opencontainers.image.description="Ground vehicle dynamics validation toolkit "
