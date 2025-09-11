# Changelog

- Added a suite of tests for all scenarios to validate the code automatically
- Added a custom autonomous driver accepting (velocity/steering angle) pairs
- Added support for delayed/throttled visualization
- Added support for large SCM grids
- Removed legacy code that was unsupported/not up to date (steady state conering, ride quality)
- Removed ROS-based speed and steering controllers in favour of the Chrono-based custom one above
- Added generation of a Bezier path for the global path in autonomous navigation maneuvers
- Added the computation of "gates" along generated Bezier paths
- Added automatic obstacles generation (with terrain height initialization)
- Fixed all scenarios and provided examples for each case
- Updated documentation pages to mirror the latest API and structure (in progress)
- Added docstrings for some of the code (in progress)
- Fixed all Docker images and documented the deployment