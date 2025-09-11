# Configuration schema

This is the schema of the configuration file.

## Executable parameters

```{csv-table} Executable parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Parameter,Description,Allowed values,Default value
log.level,Log verbosity level,string {'debug'; 'info'; 'warning'; 'error'},info
```

## Terrain parameters

```{csv-table} Terrain parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Parameter,Description,Allowed values,Default value
terrain.type,Type of terrain,string {'rigid'; 'scm'; 'crg'},rigid
terrain.rigid.patches.[].material.frictionCoefficient,Friction coefficient of the terrain patch,double,0.85
terrain.rigid.patches.[].position.x,X coordinate of the terrain patch center,double,0.0
terrain.rigid.patches.[].position.y,Y coordinate of the terrain patch center,double,0.0
terrain.rigid.patches.[].size.length,Length of the terrain patch,double,0.0
terrain.rigid.patches.[].size.width,Width of the terrain patch,double,0.0
terrain.rigid.patches.[].heightmap.file,Path to the height map file. Only used for rigid terrain patches.,string,
terrain.rigid.patches.[].heightmap.lowest,Minimum height in the terrain heightmap,double,0.0
terrain.rigid.patches.[].heightmap.highest,Maximum height in the terrain heightmap,double,1.0
terrain.rigid.patches.[].visualization.texture.path,Path to the terrain patch texture file,string,
```

## Simulation parameters

```{csv-table} Simulation parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Parameter,Description,Allowed values,Default value
simulation.timeStep,Simulation time step in seconds,double,0.0005
simulation.warmupTime,Time before vehicle commands are issued and simulation output is recorded,double,2.0
simulation.endTime,Simulation end time in seconds,double,10.0
```

## Driver parameters

```{csv-table} Driver parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Parameter,Description,Allowed values,Default value
driver.timeToMaximumThrottle,The number of seconds required to reach the maximum throttle,double,10.0
driver.maximumThrottle,The maximum throttle effort at steady state,double,1.0
driver.type,The type of driver model used,straightLine,straightLine
```

## Controllers parameters

```{csv-table} Controllers parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Parameter,Description,Allowed values,Default value
speedController.proportionalGain,Longitudinal speed controller proportional gain,double,0.4
speedController.integralGain,Longitudinal speed controller integral gain,double,0.0
speedController.derivativeGain,Longitudinal speed controller derivative gain,double,0.0
steeringController.proportionalGain,Lateral steering controller proportional gain,double,1.5
steeringController.integralGain,Lateral steering controller integral gain,double,0.0
steeringController.derivativeGain,Lateral steering controller derivative gain,double,0.0
steeringController.lookaheadDistance,Lateral steering controller path look-ahead distance,double,5.0
```

## Vehicle parameters

```{csv-table} Vehicle parameters
:widths: 20, 60, 10, 10
:header-rows: 1


Parameter,Description,Allowed values,Default value
vehicle.system,Vehicle system override template path,string,
vehicle.engine,Vehicle engine subsystem override template path,string,
vehicle.transmission,Vehicle transmission subsystem override template path,string,
vehicle.frontTires,Vehicle front tires subsystem override template path,string,
vehicle.rearTires,Vehicle rear tires subsystem override template path,string,
solver.maximumIterations,Maximum number of iterations in the physics solver,int,200
```

## Output parameters

```{csv-table} Output parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Parameter,Description,Allowed values,Default value
output.frequency,Write frequency for the simulation output,double,200.0
output.path,Path for the simulation output,double,./
output.filename,OlavDLC,string,results
output.addTimestamp,Append timestamp to the simulation results file,bool,true
output.useNameGenerator,Append an auto-generated human-readable name,bool,true
```

## Visualization parameters

```{csv-table} Output parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Parameter,Description,Allowed values,Default value
visualization.enabled,Whether or not to display the simulation visualization window,bool,false
visualization.fps,Number of frame per second for the visualization (expressed in simulation time),double,30.0
visualization.wireframe,Show mesh wireframe instead of mesh shaded view for visualization objects,bool,false
visualization.fullscreen,Whether or not to display the simulation visualization window in full screen mode,bool,false
visualization.shadows,Whether or not to render shadows in the simulation visualization (warning: when enabled, this option negatively impacts performance!),bool,false
visualization.gui.enabled,Whether or not to display the GUI overlay,bool,true
visualization.gui.fontSize,Font size used in the GUI overlay,double,16.0
```