# Examples

This page provides a description of the examples cases included in the DYNO
repositories. These examples are a good starting point for new DYNO users who
would like to use the pre-implemented scenarios and are not looking into
implementing their own custom simulations.

The examples are stored under the `examples/` folder in the root of the
repository. The eamples cover the usage of DYNO as a standalone application, as
part of design exploration studies using Dakota, and as a simulation node in
ROS environments.

:::{caution}
Docker users should use the image tagged `runtime` to run the examples.
If you are a developer and would like to run an example from the 'devel` image,
you might have to ensure DYNO has been successfully built inside the container
and provide a symlink to the DYNO binary as follows:

```shell
ln -s ~/workspace/build/bin/dyno ~/.local/bin/dyno
```

:::

:::{caution}
Before running examples using the Python interface, the `pydyno` python package must be
available globally or in the activated virtual environment. To activate the
bundled Poetry virtual environment, run the following commands from the root of
the repository:

```shell
poetry install
$(poetry env activate)
```

:::

## Standalone examples

:::{note}
> This example files are located under `examples/standalone`.
:::

The standalone examples show a minimal example of running DYNO with a custom
scenario configuration file from command line. The examples are bundled with a
Python script `launch.py` that spawns the DYNO subprocess with the scenario
matching the folder name and feeds the `config.json` configuration file.

You can directly modify the `launch.py` script to launch different scenarios or
`config.json` to use different scenario options.

The following standalone examples are included:

* Straight line acceleration
* Terrain mobility
* Wall-to-wall turn
* Autonomous navigation
* Split surface
* Sinusoidal steering
* Double lane change
* Straight line braking
* Sideslope stability

## ROS examples

:::{note}
> This example files are located under `examples/ros`.
:::

:::{caution} Docker users can use the image tagged as `ros` to run these
examples. The examples can be launched **from the host system** using `docker
compose`. :::

* `node/`: contains a minimal set up to span a DYNO simulation node using
Docker Compose.
* `autonomous_navigation/` contains an example using DYNO and ROS for an
obstacle avoidance maneuver reliabilty study using an external controller. You
will need to modify `docker-compose.yml` to spawn a ROS node with your custom
controller for this example.

## Dakota examples

:::{note}
> This example files are located under `examples/dakota`.
:::

To launch a Dakota examples, you may use the bundled shell script `launch.sh`
with one of the provided verbs:

* `run` runs the case
* `clean` removes all results generated from the previous runs
* `save` creates a timestamped archive with all generated results before cleaning the working directory

Each case contains the following key files and folders:

* `include` contains all configuration files. If you would like to add a
  substitution variable to the study or specify a different option for the
  scenario, you may edit any of the configuration files under this folder.
* `bin` includes all helper scripts for the Dakota run. Most examples will include:
  * `case.py`: the entrypoint script to launch DYNO via `pydyno` - you should
      never edit this file.
  * `analysis_driver.sh`: the script to run the DYNO simulation.
  * `input_filter.sh`: the script to run the Dakota preprocessor.
  * `output_filter.sh`: the script to run the DYNO postprocessor.
* `case.in`: the Dakota case file. If you would like to change the study type,
  add a variable or change from sequential to parallel execution, this is where
  you can do it.

### Double lane change

This is an uncertainty quantification study on a double lane change manoeuvre.

### Grade climbing

This is a design exploration study on a grade climbing manoeuvre.

### Powertrain (design/powertrain)

This example shows a straight line acceleration vector parameter study in Dakota
to characterize the acceleration performance with different powertrain
configurations.

In particular, the example shows how to substitute Dakota variables into custom
vehicle JSON configuration files. The preprocessor links both the scenario
configuration file `include/case.tpl.json` and the transmission definition file
`include/transmission.tpl.json`. Using the helper script `bin/get_cwd.py`, the
preprocessor substitutes the absolute path to the transmission configuration
file such that DYNO may now load the modified configuration for each individual
run.

This basic example runs on a flat surface and shows how a modified transmission
shift points map affects the transient response of the vehicle.
