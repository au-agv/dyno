# Architecture and components

This page describes some of the key functionality available in DYNO and how to
put it to use. It is meant as a starting point for new users that with get
acquainted with the architecture and main features of DYNO without directly
referring to the API.

## Scenarios

DYNO can simulate a number of pre-defined scenarios. Each scenario defines a
specific environment setup and event triggers suited for a specific analysis.
Each scenarios exposes a specific set of user-defined parameters to tweak
the analysis while still retaining the overall structure of the challenge.

You can find a list of the scenarios currently available in the
[Scenarios](#scenarios) page.



### Configuring scenario options

The main executable provides a parameter to specify a configuration file in JSON
format. This is the preferred way of specifying simulation options in DYNO, as
opposed to implementing custom scenarios using the API. Most scenario options
are exposed through the JSON configuration file and documented here.

The configuration file allows user to tweak parameters both agnostic and
specific to a given scenario.

Note that you may also run a scenario without specifying a configuration file.
When no configuration file is specified at runtime, a default configuration file
is loaded, the options of which can be inspected under the
`data/templates/scenarios` folder relative to the repository root.

### Search paths for models and assets

Assets for each scenario (including vehicle meshes, terrain textures and
environment object meshes) are handled through Project Chrono. The interface
specifies two main paths, one for generic assets for the Chrono core module, and
another one for assets for the Chrono::Vehicle module. By default, DYNO looks
for Chrono assets in the Chrono installation directories.

It is often the case the user would like to run a vehicle model included in DYNO
and not part of the Chrono build installed on the system. In that case, the
`paths.root` configuration option can be set to "dyno", and DYNO will look for
all assets in the DYNO folders instead. It is important to note that all Chrono
assets are shadowed in the process, and elements such as default terrain
textures or skybox texture will therefore not be available.

If users would like to include their own asset search paths instead, the
`paths.root` option can be set to "custom", and two assets search paths (one for
the Chrono core module and one for the Chrono::Vehicle module) can be specified
under `paths.custom.data` and `paths.custom.vehicle`.

### Logging

DYNO supports different logging levels with varying level of verbosity. By
default, DYNO shows log entries with severity marked as `INFO` or above. This is
the recommended setting for single runs, as it allows for easy validation of the
simulation parameters. However, when running multiple parallel instances of
DYNO, this will result in flooding the console with unsorted information
messages. In these cases, we recommend using a log severity level of `WARN` or
above.

You can change the log level by specifying the `log.level` key in the
configuration file to one of the accepted log levels. See the [Configuration]
for a list of the available options.


### Visualization

DYNO supports realtime visualization through the Project Chrono modules
Chrono::Irrlicht and Chrono::VSG (optional). The bundled Docker images include
both options, and we recommend using the VSG engine whenever possible as the
Irrlicht module is now considered legacy in Project Chrono.

Depending on your simulation, visualization may be very costly. This is
especially the case for simulation involving large meshes (high visual-fidelity
3D vehicle models, large deformable terrains, etc.). You have the option to turn
off visualization entirely, or limit the frequency at which the visualization
engine is updating the viewport. Moreover, the frequency at which the update
operation is called may be referenced relative to system time or simulation
time:

* You should use **system time** when your simulation runs slower than realtime
  and you require responsive camera controls, or in most cases when your
  visualization has a negligible computational cost.

* You should use **simulation time** when your simulation runs slower than
  realtime and you do not require responsive camera controls, or in all cases
  when your visualization has a very high computational cost (e.g. large SCM
  terrain meshes).

Also note that, while preferred, the VSG engine has a slightly higher overhead
than the Irrlicht engine when initializing large meshes for the first time.

```{csv-table} Visualization options
:widths: 20, 50, 15, 15
:header-rows: 1

Key, Description, Type [Range], Default value
`visualization/enabled`, Whether or not to run the visualization engine, bool, false
`visualization/useSystemClock`, Whether or not to use the system clock (as opposed to simulator clock) when defining frame rates, bool, false
`visualization/fps`, The rate at which new frames are rendered, float(0.0 - +inf), 30.0
`visualization/light`, Set the light intensity, float (0.0 - +inf), 0.5
`visualization/skybox`, Whether or not to render the default skybox, bool, true
`visualization/fullscreen`, Whether or not to launch the window in fullscreen mode, bool, false
`visualization/windowSize/width`, The width (in pixels) of the visualization window, int (0 - +inf), 1920
`visualization/windowSize/height`, The height (in pixels) of the visualization window, int (0 - +inf), 720
`visualization/gui/enabled`, Whether or not to display the graphical user interface overlay, bool, true
`visualization/gui/fontSize`, The font size used in the graphical user interface overlay, float(0.0 - +inf), 16.0
`visualization/shadows`, Whether or not to render shadows (be aware that this comes at a very high computational cost for the VSG engine!), bool, false
`output/visualization/enabled`, Whether or not to save the rendered frames to image, bool, false
`output/visualization/path`, The absolute or relative path to the folder where the rendered frames will be saved, string, ./
`visualization/backgroundColor`, Color used for the scene background (only has effect when the skybox is disabled), vector[3], [0.22 0.43 0.61]
```


```{csv-table} Command line options
:widths: 20, 35, 15, 15, 15
:header-rows: 1

Argument,Description,Required,Available values,Default value
--input-filter,If specified, run in Dakota input filter mode.,No,,false
--parameters-file,Specify the path of the Dakota parameters file.,No,,-
--launch-file,Specify the ROS launch file,No,,-
--output-files,Specify the output files of the Dakota input filter,No,,-
--python-includes,Specify the Python includes for the Dakota preprocessor,No,,-
--output-filter,If specified, run in Dakota output filter mode.,No,,false
--mode,Mode to run DYNO in,No,{standalone,ros},standalone
--driver,If specified, run in Dakota analysis driver mode,No,,
--asynchronous,Specify the number of asynchronous threads to run in Dakota analysis driver mode,No,,
--scenario,Specify the scenario to run,Yes,{},
--config,Specify the configuration file to use,Yes,{},
--log,Specify the log level,No,{},warn
```