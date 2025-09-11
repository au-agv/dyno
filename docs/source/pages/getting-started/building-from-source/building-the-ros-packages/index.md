# Building the ROS packages

:::{note}
To run the DYNO ROS packages, you will need a working installation of ROS 2
Humble. Please note that other (newer or older) releases of ROS 2 may also work,
but they have not been tested.
:::

:::{note}
You will also need a working installation of DYNO, available globally on your
system. Refer to the [Getting started](../../index.md#getting-started) page for more
information.
:::

Assuming your `colcon` workspace is located under `{COLCON_WORKSPACE}` and your
DYNO repository root is located under `{DYNO_REPOSITORY}`, you can symlink the
packages with the following command:

```shell
ln -s {DYNO_REPOSITORY}/ros {COLCON_WORKSPACE}/src/dyno
```

Then, install the required dependencies using `rosdep` (you may have to
initialize `rosdep` as described
[here](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html))

```shell
rosdep install --from-paths src -y --ignore-src
```

You are now ready to build the DYNO ROS packages using `colcon`:

```shell
colcon build --packages-select dyno_interfaces dyno_simulation dyno_utilities
```
