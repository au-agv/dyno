ARG TARGET=cpu
ARG CHRONO_PATH=/usr/local/src/chrono
ARG CHRONO_COMMIT=1366b629b77390b60cd5446c2e973f8081fee4b0
ARG BUILD_TYPE=RelWithDebInfo
ARG DEBIAN_FRONTEND=noninteractive
ARG LOCALE=en_US
ARG USER=robot

###############################################################################
# Compile and install Project Chrono                                          #
###############################################################################

FROM ubuntu:22.04 AS cpu

LABEL org.opencontainers.image.title="DYNO (devel)"

ARG CHRONO_PATH
ARG CHRONO_COMMIT
ARG BUILD_TYPE
ARG DEBIAN_FRONTEND
ARG LOCALE
ARG USER

RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        build-essential \
        ca-certificates \
        cmake \
        freeglut3-dev \
        git \
        libeigen3-dev \
        libgl-dev \
        libirrlicht-dev \
        libvulkan-dev \
        libx11-dev \
        libxxf86vm-dev \
        ninja-build \
        python3 \
        python-is-python3 && \
    git clone https://github.com/projectchrono/chrono ${CHRONO_PATH} && \
    git -C ${CHRONO_PATH} checkout ${CHRONO_COMMIT} && \
    cd ${CHRONO_PATH}/contrib/build-scripts/linux && \
    sed -i 's/15.4.0/15.0.0/g' buildVSG.sh && \
    chmod +x buildVSG.sh && \
    ./buildVSG.sh /opt/vsg && \
    cp -R /opt/vsg/** /usr/local && \
    rm -rf /opt/vsg && \
    mkdir -p ${CHRONO_PATH}/build && \
    cd ${CHRONO_PATH}/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_DEMOS=OFF \
        -DCH_ENABLE_MODULE_IRRLICHT=ON \
        -DCH_ENABLE_MODULE_VEHICLE=ON \
        -DCH_ENABLE_MODULE_POSTPROCESSING=ON \
        -DCH_ENABLE_MODULE_VSG=ON && \
    cmake --build . -j$(nproc) --target install && \
    rm -rf ${CHRONO_PATH} && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

FROM nvidia/cuda:12.3.2-cudnn9-devel-ubuntu22.04 AS cuda

LABEL org.opencontainers.image.title="DYNO (devel-cuda)"

ARG CHRONO_PATH
ARG CHRONO_COMMIT
ARG BUILD_TYPE
ARG CUDA_ARCHITECTURES=86
ARG DEBIAN_FRONTEND
ARG LOCALE
ARG USER

COPY --chmod=755 docker/build/standalone/devel/third_party/NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh /usr/local/bin/install-optix.sh

RUN mkdir -p /usr/local/optix && \
    /usr/local/bin/install-optix.sh --skip-license --prefix=/usr/local/optix

RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        build-essential \
        cmake \
        freeglut3-dev \
        git \
        libeigen3-dev \
        libgl-dev \
        libirrlicht-dev \
        libvulkan-dev \
        libx11-dev \
        libxxf86vm-dev \
        ninja-build \
        python3 \
        python-is-python3 && \
    git clone https://github.com/projectchrono/chrono ${CHRONO_PATH} && \
    git -C ${CHRONO_PATH} checkout ${CHRONO_COMMIT} && \
    cd ${CHRONO_PATH}/contrib/build-scripts/linux && \
    sed -i 's/15.4.0/15.0.0/g' buildVSG.sh && \
    chmod +x buildVSG.sh && \
    ./buildVSG.sh /opt/vsg && \
    cp -R /opt/vsg/** /usr/local && \
    rm -rf /opt/vsg && \
    cd ${CHRONO_PATH} && \
    mkdir -p ${CHRONO_PATH}/build && \
    cd ${CHRONO_PATH}/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_DEMOS=ON \
        -DCH_ENABLE_MODULE_IRRLICHT=ON \
        -DCH_ENABLE_MODULE_POSTPROCESSING=ON \
        -DCH_ENABLE_MODULE_SENSOR=ON \
        -DCH_ENABLE_MODULE_VEHICLE=ON \
        -DCH_ENABLE_MODULE_VSG=ON \
        -DCUDA_CUDA_LIBRARY=/usr/local/cuda/lib64/stubs/libcuda.so \
        -DCHRONO_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES} \
        -DCH_OptiX_INSTALL_DIR=/usr/local/optix \
        -DOptiX_ROOT_DIR=/usr/local/optix \
        -DCH_USE_SENSOR_NVRTC=OFF && \
    cmake --build . -j$(nproc) --target install && \
    rm -rf ${CHRONO_PATH} && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

FROM ${TARGET} AS common

ARG DEBIAN_FRONTEND
ARG BUILD_TYPE
ARG LOCALE
ARG USER

###############################################################################
# Compile and install HighFive                                                #
###############################################################################

ARG HIGHFIVE_PATH="/usr/local/src/highfive"

RUN apt-get update && \
    apt-get install --no-install-recommends --yes libhdf5-dev && \
    git clone https://github.com/highfive-devs/highfive ${HIGHFIVE_PATH} && \
    mkdir -p ${HIGHFIVE_PATH}/build && \
    cd ${HIGHFIVE_PATH}/build && \
    cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DBUILD_TESTING=OFF -DHIGHFIVE_UNIT_TESTS=OFF && \
    cmake --build . -j$(nproc) --target install && \
    rm -rf ${HIGHFIVE_PATH} && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

###############################################################################
# Compile and install DAKOTA                                                  #
###############################################################################

ARG DAKOTA_PATH="/usr/local/src/dakota"

RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        gfortran \
        libboost-filesystem-dev \
        libboost-program-options-dev \
        libboost-regex-dev \
        libboost-serialization-dev  \
        liblapack-dev \
        libhdf5-dev \
        python3-h5py \
        python3-numpy \
        python3-scipy && \
        git clone --branch v6.21.0 https://github.com/snl-dakota/dakota.git ${DAKOTA_PATH} && \
    cd ${DAKOTA_PATH} && \
    git submodule init packages/external && \
    git submodule init packages/pecos && \
    git submodule update && \
    git clone https://github.com/snl-dakota/surfpack.git ${DAKOTA_PATH}/packages/surfpack && \
    cd ${DAKOTA_PATH}/packages/surfpack && \
    git checkout 30b316b && \
    mkdir -p ${DAKOTA_PATH}/build && \
    cd ${DAKOTA_PATH}/build && \
    cmake .. -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DDAKOTA_HAVE_HDF5=ON -DDAKOTA_PYTHON=ON -DDAKOTA_PYTHON_SURROGATES=ON && \
    cmake --build . -j$(nproc) --target install && \
    rm -rf ${DAKOTA_PATH} && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

COPY --chmod=755 docker/build/standalone/devel/third_party/dakota-pyproject.toml /usr/local/share/dakota/Python/dakota/pyproject.toml

###############################################################################
# Compile and install ZMQPP                                                   #
###############################################################################

ARG ZMQPP_PATH="/usr/local/src/zmqpp"

RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        libzmq3-dev && \
    git clone --branch 4.2.0 https://github.com/zeromq/zmqpp.git ${ZMQPP_PATH} && \
    cd ${ZMQPP_PATH} && \
    mkdir -p ${ZMQPP_PATH}/build && \
    cd ${ZMQPP_PATH}/build && \
    cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE} && \
    cmake --build . -j$(nproc) --target install && \
    rm -rf ${ZMQPP_PATH}  && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


###############################################################################
# Install DYNO dependencies                                                   #
###############################################################################

RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        doxygen \
        gdb \
        libgtest-dev \
        libopencv-dev \
        libspdlog-dev \
        locales \
        nlohmann-json3-dev \
        pipx \
        graphviz \
        # Additional dependencies for matplotlib plotting
        libx11-dev \
        libxcb-cursor0 \
        python3 \
        python3-tk \
        qtwayland5 \
        texlive \
        texlive-latex-extra \
        texlive-fonts-recommended \
        dvipng \
        cm-super && \
    echo "LC_ALL=${LOCALE}.UTF-8" >> /etc/environment && \
    echo "${LOCALE}.UTF-8 UTF-8" >> /etc/locale.gen && \
    echo "LANG=${LOCALE}.UTF-8" > /etc/locale.conf && \
    locale-gen && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

ENV LANG=${LOCALE}.UTF-8
ENV LANGUAGE=${LOCALE}:en
ENV LC_ALL=${LOCALE}.UTF-8

###############################################################################
# Add unprivileged user                                                       #
###############################################################################

# Add a container user, group and home working directory
RUN groupadd --gid 1000 ${USER} && \
    useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash ${USER} && \
    echo "export PATH='/home/${USER}/.local/bin:${PATH}'" >> /home/${USER}/.profile

# Set the working directory to the home folder of the container user
WORKDIR /home/${USER}

# Set the default container user
USER ${USER}

# Set the default shell to a Bash login shell
SHELL ["/bin/bash", "-l", "-c"]

RUN pipx install poetry

###############################################################################
# Add container labels                                                        #
###############################################################################

LABEL org.opencontainers.image.authors="Dario Sirangelo (dev@dariosirangelo.me)"
LABEL org.opencontainers.image.url=https://github.com/au-agv/dyno
LABEL org.opencontainers.image.documentation=https://au-agv.github.io/dyno
LABEL org.opencontainers.image.source=https://github.com/au-agv/dyno/tree/main/docker/build/standalone/devel
LABEL org.opencontainers.image.version=main
LABEL org.opencontainers.image.licenses=MIT
LABEL org.opencontainers.image.description="Ground vehicle dynamics validation toolkit "

