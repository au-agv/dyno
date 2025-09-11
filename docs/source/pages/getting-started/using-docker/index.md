# Using Docker

## Pulling the image

The prebuilt image is available on the GitHub container registry. You can pull
the image with a tag matching your desired application, or you can build the
Docker image locally.

There are four different Docker image variants:

* `devel`, suitable for development, DYNO must be built manually inside the
  container.
* `runtime`, suitable for deployment, DYNO is built and its executable is
  available in PATH.
* `ros-devel`, suitable for development of the ROS node. The node must be built
  manually inside the container.
* `ros-runtime`, suitable for deployment of the ROS node. The DYNO ros node is
  built and available in the local ROS workspace.

Moreover, all images are available in their default and CUDA (labelled with the
trailing `-cuda`) variants. The latter are much larger, but add support for
sensors and GPU-accelerated visualization for compatible hosts. Note that `ros`
images inherit from the `cuda` image, as they require sensor simulation.

* The latest built version is labelled as `-stable`.
* Specific versions are labelled based on to the commits they are build against
  (`<hash>`).

To pull a specific Docker image, use the following command:

```shell
docker pull gchr.io/aarhus-robotics/dyno:{tag}}
```

Below is a list of the available tag and a brief description of each.

:::{note}
If you are an end-user looking to run DYNO, the `runtime` image is suitable for
your application.
:::


| Tag | Description |
| --- | --- |
| `devel`, `devel-<hash>`, `devel-stable` | Development image |
| `devel-cuda`, `devel-<hash>-cuda`, `devel-stable-cuda` | Development image, with CUDA support |
| `runtime`, `runtime-<hash>`, `runtime-stable` | Runtime image with latest stable version |
| `cuda`, `runtime-cuda`, `runtime-<hash>-cuda`, `runtime-stable-cuda` | Runtime image with latest stable version, with CUDA support |
| `ros-devel`, `ros-devel-<hash>`, `ros-devel-stable` | ROS interface node image |
| `ros`, `ros-runtime`, `ros-runtime-<hash>`, `ros-runtime-stable` | ROS interface node image, with CUDA support |

## Running the image

You can spawn a new interactive shell with `dyno` in a container by running:

```shell
docker run -it ghcr.io/aarhus-robotics/dyno:runtime
```

You may then use the command `dyno` to run your scenario. For example, to
run the straight line acceleration scenario with default parameters:

```shell
dyno --scenario straightLineAcceleration
```

Refer to the [user guide](../../user-guide/index.md#usage-guide) for more details on the command line
interface options and arguments.