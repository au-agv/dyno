FROM ghcr.io/aarhus-robotics/dyno:runtime-stable-cuda

ARG DEBIAN_FRONTEND=noninteractive
ARG LOCALE=en_US
ARG ROS_DISTRIBUTION=humble

USER root

###############################################################################
# Install ROS                                                                 #
###############################################################################

# Install ROS dependencies and set up locales
RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        curl \
        locales \
        software-properties-common && \
    locale-gen ${LOCALE} ${LOCALE}.UTF-8 && \
    update-locale LC_ALL=${LOCALE}.UTF-8 LANG=${LOCALE}.UTF-8\
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 repository and install the distribution
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install --no-install-recommends --yes \
        ros-dev-tools \
        ros-${ROS_DISTRIBUTION}-ackermann-msgs \
        ros-${ROS_DISTRIBUTION}-desktop \
        ros-${ROS_DISTRIBUTION}-rosbag2-storage-mcap && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

###############################################################################
# Add unprivileged user                                                       #
###############################################################################

USER robot

RUN echo "export LANG=${LOCALE}.UTF-8" >> /home/robot/.profile && \
    echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.bash" >> /home/robot/.profile && \
    #echo "source /home/robot/ROS/build/setup.bash 2> /dev/null" >> /home/robot/.profile && \
    echo "export ROS_LOG_DIR=\"/home/robot/ROS/log\"" >> /home/robot/.profile

###############################################################################
# Set up the ROS workspace                                                    #
###############################################################################

RUN mkdir -p ${HOME}/ROS/{build,install,log,src}

VOLUME ["/home/robot/ROS/build", "/home/robot/ROS/install"]

###############################################################################
# Set up container entrypoint                                                 #
###############################################################################

#COPY --chown=1000:1000 --chmod=755 docker/build/ros/devel/entrypoint.sh /home/robot/entrypoint.sh
#CMD ["/bin/bash", "-l", "-c", "/home/robot/entrypoint.sh"]

###############################################################################
# Add container labels                                                        #
###############################################################################

LABEL org.opencontainers.image.authors="Dario Sirangelo (dsi@aarhusrobotics.com)"
LABEL org.opencontainers.image.url=https://github.com/aarhus-robotics/dyno
LABEL org.opencontainers.image.documentation=https://docs.aarhusrobotics.com/dyno
LABEL org.opencontainers.image.source=https://github.com/aarhus-robotics/dyno/tree/main/docker/build/ros/devel
LABEL org.opencontainers.image.version=main
LABEL org.opencontainers.image.licenses=MIT
LABEL org.opencontainers.image.title="DYNO (ros-devel)"
LABEL org.opencontainers.image.description="Ground vehicle dynamics validation toolkit "
