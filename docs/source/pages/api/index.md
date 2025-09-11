:html_theme.secondary_sidebar_items.[]:

# API

This is the reference page for the DYNO C++ and Python API.

## Library (C++)

### Drivers

#### AccelerationBasedDriver

```{eval-rst}
.. doxygenclass:: DYNO::Drivers::AccelerationBasedDriver
```

#### AutonomousDriver

```{eval-rst}
.. doxygenclass:: DYNO::Drivers::AutonomousDriver
```

#### VelocityBasedDriver

```{eval-rst}
.. doxygenclass:: DYNO::Drivers::VelocityBasedDriver
```

### Interfaces

#### JSONConfiguration

```{eval-rst}
.. doxygenclass:: DYNO::Interfaces::JSONConfiguration
```

#### JSONTerrainOutput

```{eval-rst}
.. doxygenclass:: DYNO::Interfaces::JSONTerrainOutput
```

#### ZMQInterface

```{eval-rst}
.. doxygenclass:: DYNO::Interfaces::ZMQInterface
```

#### ZMQSocket

```{eval-rst}
.. doxygenclass:: DYNO::Interfaces::ZMQSocket
```

### Math

#### PoissonDiskSampler1D

```{eval-rst}
.. doxygenclass:: DYNO::Math::PoissonDiskSampler1D
```

#### PoissonDiskSampler2D

```{eval-rst}
.. doxygenclass:: DYNO::Math::PoissonDiskSampler2D
```

#### PoissonDiskSampler

```{eval-rst}
.. doxygenclass:: DYNO::Math::PoissonDiskSampler
```

#### PoissonPoint1D

```{eval-rst}
.. doxygenclass:: DYNO::Math::PoissonPoint1D
```

#### PoissonPoint2D

```{eval-rst}
.. doxygenclass:: DYNO::Math::PoissonPoint2D
```

#### PoissonPointState

```{eval-rst}
.. doxygenenum:: DYNO::Math::PoissonPointState
```

### Models

#### AerodynamicProperties

```{eval-rst}
.. doxygenclass:: DYNO::Models::AerodynamicProperties
```

#### CameraParameters

```{eval-rst}
.. doxygenstruct:: DYNO::Models::CameraParameters
```

#### ControllerTuning

```{eval-rst}
.. doxygenclass:: DYNO::Models::ControllerTuning
```

#### ImuParameters

```{eval-rst}
.. doxygenstruct:: DYNO::Models::ImuParameters
```

#### LidarParameters

```{eval-rst}
.. doxygenstruct:: DYNO::Models::LidarParameters
```

#### Olav

```{eval-rst}
.. doxygenclass:: DYNO::Models::Olav
```

#### SpeedControllerTuning

```{eval-rst}
.. doxygenclass:: DYNO::Models::SpeedControllerTuning
```

#### SteeringControllerTuning

```{eval-rst}
.. doxygenclass:: DYNO::Models::SteeringControllerTuning
```

#### TrackedVehicle

```{eval-rst}
.. doxygenclass:: DYNO::Models::TrackedVehicle
```

#### Vehicle

```{eval-rst}
.. doxygenclass:: DYNO::Models::Vehicle
```

#### WheeledVehicle

```{eval-rst}
.. doxygenclass:: DYNO::Models::WheeledVehicle
```

### Sensors

#### IMU

```{eval-rst}
.. doxygenclass:: DYNO::Sensors::IMU
```

#### LidarXYZI

```{eval-rst}
.. doxygenclass:: DYNO::Sensors::LidarXYZI
```

#### RGBACamera

```{eval-rst}
.. doxygenclass:: DYNO::Sensors::RGBACamera
```

### Serialization

#### HDF5Serializer

```{eval-rst}
.. doxygenclass:: DYNO::Serialization::HDF5Serializer
```

#### JSONSerializer

```{eval-rst}
.. doxygenclass:: DYNO::Serialization::JSONSerializer
```

#### Serializer

```{eval-rst}
.. doxygenclass:: DYNO::Serialization::Serializer
```

### Simulation

#### AutonomousNavigation

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::AutonomousNavigation
```

#### AutonomousVehicleSimulation

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::AutonomousVehicleSimulation
```

#### DoubleLaneChange

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::DoubleLaneChange
```

#### Obstacle

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::Obstacle
```

#### Path

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::Path
```

#### SideslopeStability

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::SideslopeStability
```

#### SinusoidalSteering

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::SinusoidalSteering
```

#### SplitSurface

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::SplitSurface
```

#### StraightLineAcceleration

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::StraightLineAcceleration
```

#### StraightLineBraking

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::StraightLineBraking
```

#### TerrainMobility

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::TerrainMobility
```

#### VehicleSimulation

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::VehicleSimulation
```

#### WallToWall

```{eval-rst}
.. doxygenclass:: DYNO::Simulation::WallToWall
```

### Visualization

#### IrrlichtTracked

```{eval-rst}
.. doxygenclass:: DYNO::Visualization::IrrlichtTracked
```

#### IrrlichtWheeled

```{eval-rst}
.. doxygenclass:: DYNO::Visualization::IrrlichtWheeled
```

#### Irrlicht

```{eval-rst}
.. doxygenclass:: DYNO::Visualization::Irrlicht
```

#### VulkanSceneGraph

```{eval-rst}
.. doxygenclass:: DYNO::Visualization::VulkanSceneGraph
```

#### Wrapper

```{eval-rst}
.. doxygenclass:: DYNO::Visualization::Wrapper
```

## ROS (C++)

### Native

```{eval-rst}
.. doxygenclass:: DYNO::ROS::Native::NativeNode
```

## Utilities (Python)

### Launch

```{eval-rst}
.. automodule:: pydyno.launch
```

### Launcher

```{eval-rst}
.. automodule:: pydyno.launcher
```

### Postprocess

```{eval-rst}
.. automodule:: pydyno.postprocess
```

### Postprocessor

```{eval-rst}
.. automodule:: pydyno.postprocessor
```

### Utilities

```{eval-rst}
.. automodule:: pydyno.tooling
```
