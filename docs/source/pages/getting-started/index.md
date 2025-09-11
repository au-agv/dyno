# Getting started

```{toctree}
:hidden:
using-docker/index.md
building-from-source/index.md
```

## Overview

This section provides information on how to install DYNO and integrate it in
your own application.

The recommended installation method is [using the Docker
image](./using-docker/index.md#using-docker). There are Docker image suitable
for developers, end users or specific software integrations (e.g. ROS). If you
are unable to use the Docker image, you can [build DYNO from
sources](./building-from-source/index.md#building-from-source).

## Quick start

If you are unsure about your specific application and just want to try and
quicky run DYNO on your machine, you can run the following Docker command to get
the latest runtime:

```shell
docker run -it ghcr.io/aarhus-robotics/dyno:runtime
```

Once inside the container, launch a simple simulation using default options with
the following command:

```shell
cd examples/standalone/straight_line_acceleration
./launch.py
```

After the simulation is completed, analyze the results using the command:

```shell
dynopost --file straightLineAcceleration.h5 plot velocity.linear.x --show
```
