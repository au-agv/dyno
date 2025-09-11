# Postprocessing

## Overview

The DYNO postprocessor can be used in two ways:

- Using the command line interface, which provides a limited set of features to
  quickly generate plots or reports with standard quantities and format.
- The Python API for the postprocessor, which provides full access to all
  plotting and reporting features, but requires users to write their own
  postprocessing script in Python.

## Postprocessor CLI options

### Plotter

The plotter subparser is accessed through the verb `plot`. It processes single
HDF5 results file from a provided workspace folder and allows to plot and save
reports which include time series from the results file.

Argument,Required,Description,Available values (Default)
`entry`,Yes,"The group entry to be plotted, relative to the HDF5 file root, with nested entries separated by a dot (e.g. `velocity.linear.x`).",`-`
`--file`,No,"The file path where to the HDF5 results files. If no file path is provided, a dialog window for file selection is presented.",`[str]`  (`.`)
`--show`,No,"Show the generated plots in a preview window. Plots can be saved manually once shown in the preview window.",`-`
`--save [path]`,No,"Save the generated plots to individual PDF files.",`[str]` `(.)`

## Multiplotter

The multiplotter subparser is access through the verb `multiplot`.  It processes
a batch of HDF5 results file from a provided workspace folder and allows to
compute batch statistics and overlay multiple plots on a single figure.

Argument,Required,Description,Available values (Default)
`type`,Yes,"The plot type to be used.",`ensemble, histogram` (`-`)
`entry`,Yes,"The group entry to be plotted, relative to the HDF5 file root, with nested entries separated by a dot (e.g. `velocity.linear.x`).", `-`
`function`,Yes,"The function to be applied when computing statistics.",`-`
`--workspace`,No,"The workspace path where to search for HDF5 results files. If no workspace path is provided, a dialog window for folder selection is presented.",`[str]`  (`.`)
`--show`,No,"Show the generated plots in a preview window. Plots can be saved manually once shown in the preview window.",`-`
`--save [path]`,No,"Save the generated plots to individual PDF files.",`[str]` `(.)`

## Postprocessor API

```python
postprocessor = Postprocessor()
postprocessor.load_file
```

## Results file

Results are saved in JSON or HDF5 format based on user preference. Regardless of
format, all results file contain data (time series related to the vehicle) and
metadata (information related to the scenario or specific simulation run).

### Data specification

:::{note}
In addition to the data specification in this section, please refer to
additional keys relevant to each scenario.
:::

```{csv-table} Results file data
:widths: 30, 10, 60
:header-rows: 1

Key,Type,Description
pose.position.{x y z},vector[double],Vehicle center of gravity position vector (XYZ components)
pose.orientation.{x y z},vector[double],Vehicle orientation as an Euler angles sequence (XYZ components)
speed.linear.{x y z},vector[double],Vehicle linear velocity vector components in the vehicle local reference frame located at the center of gravity
speed.angular.{x y z},vector[double],Vehicle angular velocity vector components in the global reference frame
```

### Metadata specification

:::{note}
In addition to the metadatadata specification in this section, please refer to
additional keys relevant to each scenario.
:::

```{csv-table} Results file metadata
:widths: 30, 10, 60
:header-rows: 1

Key,Type,Description
vehicle,string,The name of the vehicle used in the simulation.
scenario/id,string,The identifier of the scenario used to collect the data.
scenario/parameters,dictionary,Dictionary of specific parameters for the given scenario
scenario/configuration,dictionary,Raw copy of the configuration dictionary
frequency,double,The sampling frequency for the output data
```
