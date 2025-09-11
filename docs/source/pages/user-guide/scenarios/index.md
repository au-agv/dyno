(scenarios)=
# Scenarios

## Autonomous navigation

The vehicle begins the maneuver at an initial pose located far from the first
waypoint in the path. This allows for sufficient space for the vehicle to reach
the selected target speed, which is achieved through the longitudinal speed
controller and by enforcing null steeering angle at the front axle through the
steering controller.

You can always override the initial pose of the vehicle by overriding the
OverrideInitialPose method.

Autonomous vehicle simulations have a special driver. This driver accepts
speed/steering angle control pairs.


## Double lane change

This scenario simulates a double lane change (DLC) manoeuvre following the NATO
AVTP 03-160W specification. The template supports wheeled or tracked vehicles,
rigid terrain with a single or split-surface coefficient of friction, soft
terrain via soil-contact model (SCM) with split-surface soil parameters and,
optionally, a sideslope specification.

The initial point of the manoeuvre specification is the system origin, and the
vehicle begins at a user-specified distance before said origin such that enough
space is available to the vehicle to reach a pre-determined steady-state
longitudinal speed.

```{csv-table} Double lane change scenario parameters
:file: double-lane-change-parameters.csv
:widths: 20, 60, 10, 10
:header-rows: 1

test,test,test,test
test,test,test,test
```

## Obstacle avoidance

```{csv-table} Obstacle avoidance scenario parameters
:file: obstacle-avoidance-parameters.csv
:widths: 20, 60, 10, 10
:header-rows: 1

test,test,test,test
test,test,test,test
```

## Sideslope stability

The sideslope stability scenario performs a sideslope stability test as defined
in the NATO Standard AMSP-06: Guidance for Standard Applicable to the
Development of Next Genearation NATO Reference Mobility Models (NG-NRMM)
standard.

In this scenario, the vehicle starts on a flat surface and reaches a defined
steady longitudinal speed through a straight line corridor defined by the
initial section of the manoeuvre path. The vehicle then follows a curvilinear
path resemblant of an obstacle avoidance manoeuvre, and rebounds back to a
straight line corridor immediately after.

This is a pass-fail manoeuvre, where the success criteria is the execution of
the full manoeuvre and return to straight line path.

The following parameters are exposed to the user:

```{csv-table} Sinusoidal steering scenario parameters
:widths: 20, 50, 15, 15
:header-rows: 1

Key, Description, Type, Default value
`scenario/gradePercentage`, The slope in the longitudinal direction expressed as a percentage (e.g. 24.5%), float (0.0 - +inf), 15.0
`scenario/useGradeRamp`, Whether or not the slope is gradually increased over time starting from a flat plane or immediately present at the beginning of the simulation., bool, true
`scenario/timeToMaxGrade`, Only used when `scenario/useGradeRamp` is enabled. Defines how many seconds the transition between flat plane and the defined grade will take. Note that this is interpolated using a pure sinusoidal step function., float (0.0 - +inf), 3.0
`scenario/accelerationLength`, The length of the initial straight line portion of the manoeuvre, float (0.0 - +inf), 100.0
`scenario/terrain/type`, The terrain model used in the scenario, string (rigid scm), rigid
`scenario/terrain/length`, The size of the terrain patch in the longitudinal direction, float (0.0 - +inf), 1.0e4
`scenario/terrain/width`, The size of the terrain patch in the lateral direction, float (0.0 - +inf), 20.0
`scenario/terrain/rigid/frictionCoefficient`, The friction coefficient for the rigid terrain (only used when the rigid terrain type is selected), float (0.0 - +inf), 0.85
```

## Sinusoidal steering

```{csv-table} Sinusoidal steering scenario parameters
:widths: 20, 60, 10, 10
:header-rows: 1

Key, Description, Type, Default value
test,test,test,test
```

## Split surface

## Terrain mobility


## Straight line acceleration

This scenario implements a straight line acceleration manoeuvre .

## Straight line braking

## Wall-to-wall turn
