# Interfacing with Dakota

![Dakota header](dakota.png)

:::{note}
If you would like to get started with using DYNO with Dakota, we encourage you
to read the Dakota documentation first, as you will have to set up a case study
with Dakota as your first step.
:::

:::{note}
If you are already familiar with both Dakota and DYNO, you can refer to the
`examples/dakota` folder for some reference workflows.
:::

## Introduction

DYNO provides launch, pre- and post-processing capabilities to speed up the
interfacing of your simulation with Dakota. This is achieved through the Python
interface for DYNO, which is in charge of processing the DYNO configuration
files through the Dakota preprocessor, spawning the DYNO executable with the
required parameters and performing all post-processing steps required for Dakota
to parse simulation outputs.

## An example Dakota workflow with DYNO

Here we provide an example workflow in which:

1. DYNO will feed a configuration template to the Dakota preprocessor for
   substitution
2. Dakota will launch the DYNO simulation
3. DYNO will postprocess the results to extract the quantities of interest.

In most cases, you will be using Dakota with DYNO to run a large number of
simulations. DYNO provides a postprocessor which is especially suited for output
from Dakota, as long as the Dakota case input file is configured to generate
tagged results in a work directory. To take advantage of the postprocessor, the
following keywords must be included in your Dakota case file:

```dakota
interface,
    analysis_drivers (...)
        (...)
        fork
            (...)
            file_save
            work_directory
                named `runs/run`
                directory_tag
                directory_save
                link_files `<your-configuration-file>.json
            verbatim
(...)
```

1. Create your Dakota case in a `case.in` file.
2. Create a DYNO configuration file template `configuration.tpl.json`
3. In your Dakota `case.in` case file:
    1. Set your `analysis_driver` to `dynokota analysis_driver {scenario}
       configuration.tpl.json`.
    2. Set your ìnput_filter` to `dynokota input_filter configuration.tpl.json
       --json-include {PARAMETERS}`
4. In your DYNO configuration file template:
    1. Substitute the dictionary value for the quantities of interest with the
       Dakota variables labels under curly brackets (i.e. `{variable}`).
5. (Optional) Disable visualization and logging in your `configuration.json` file
6. Launch your Dakota case with `dakota case.in`

## Postprocessing the simulation responses

In most cases, the output of DYNO are timeseries, while most Dakota simulations
expect scalar responses. To provide Dakota with the responses, the postprocessor
provides some basic filtering function that allow for applying simple statistics
to the resulting time series or select specific samples from the time series.
The filters are specified directly in the Dakota input file responses block
using a custom pattern.

The response functions are matched to a pattern `<response>:<filter>`. Here
`<response>` is a dot-separated path in the results HDF5 database file relative
to the `data` group, and `<filter>` is a function chosen from those available in
the `pydyno.processing.TimeSeriesProcessor` class.

Implementing your own filter is easy, it is sufficient to add a new private
method to `pydyno.processing.TimeSeriesProcessor` with signature
`_myfilter(self, x)`.

```python
    @staticmethod
    def _myfilter(self, x):

        return mylibrary.myfilterfunction(x)
```

You will then be able to apply the filter by specifying
`<response>:myfilter` in the Dakota case file:

```shell
(...)
responses,
    response_functions 1
    descriptors 'velocity.linear.x:myfilter'
    no_gradients
    no_hessians
```

As an example, if you would like to extract the speed of the vehicle at the end
of the simulation, you may specify a descriptor `velocity.linear.x:end` for your
response function.

## Using Dakota with DYNO and DYNO::ROS

If you are interested in running Dakota alongside DYNO ROS, you have two choices.

1. Install Dakota on the host and configure it to spawn a Docker container with
   DYNO ROS (challenging and fragile).
2. Create a specialized Docker image from
   `ghcr.io/aarhus-robotics/dyno:ros-runtime`.

In the first case, you will need some special logic to spawn a Rosbag logger and
capture end of a simulation (e.g. via the Docker Compose argument
`--abort-on-container-exit`), and if your simulations are quite brief, the
starting and stopping of the containers will be a significant bottleneck.

Because of the above, we recommend building your own Docker image instead. In
doing so, you are also able to include any other ROS package in the Docker build
(e.g. your navigation stack) as well as any custom message, services and action
definitions that may be required by the Rosbag recorder.

If you are able to provide a ROS launch file for your simulation, we provide a
launcher that can be used with Dakota specifically when working DYNO ROS. The
launcher is enabled by append the keyword `ros` to the `dynokota` executable.
The command line arguments (detailed in the executable help) follow similar
patterns as the normal `dynokota` executable:

```shell
interface,
    id_interface = 'DYNO_INTERFACE'
    analysis_drivers 'dynokota ros analysis_driver'
        input_filter 'dynokota ros input_filter case.tpl.json --json-include {PARAMETERS} --output case.json'
        output_filter 'dynokota rosoutput_filter {PARAMETERS} case.h5 {RESULTS}'
```

## Using exported Dakota surrogate models

:::{caution}
You need a Dakota Python installation to use the Dakota bindings in the Python
package. This is only required for surrogate models evaluation. There is
currently no workaround for this in Poetry, as all dependencies must be
resolvable. If you would like to use Dakota Python bindings, you will have to
explicitly uncomment the following sections under `pyproject.toml`:

```toml
# ---------------------------------------------------------------------------- #
# Dakota dependencies group
# ---------------------------------------------------------------------------- #

#[tool.poetry.group.dakota]
#optional = true

#[tool.poetry.group.dakota.dependencies.dakota]
#path = "/usr/local/share/dakota/Python/dakota"
#optional = true
```

:::
