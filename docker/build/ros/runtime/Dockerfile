FROM ghcr.io/aarhus-robotics/dyno:ros-devel

ARG ROS_DISTRIBUTION=humble
ARG BUILD_TYPE=RelWithDebInfo

COPY ros /home/robot/ROS/src/aarhus-robotics/dyno/

RUN cd /home/robot/ROS && \
    source /opt/ros/${ROS_DISTRIBUTION}/setup.bash && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

COPY --chmod='+x' docker/build/ros/runtime/entrypoint.py /home/robot/entrypoint.py

CMD ${HOME}/entrypoint.py

ENV DOCKER_ROS_ACTION=run
ENV DOCKER_ROS_PACKAGE=dyno_simulation
ENV DOCKER_ROS_EXECUTABLE=dyno_simulation_native_simulator_interface_node

###############################################################################
# Define container labels                                                     #
###############################################################################

LABEL org.opencontainers.image.source=https://github.com/aarhus-robotics/dyno.git
LABEL org.opencontainers.image.revision=main
LABEL org.opencontainers.image.name=dyno
