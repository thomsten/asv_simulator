# The asv_simulator package
This package provides an implementation of a (semi-)general implementation of a
nonlinear 3DOF underactuated surface vessel.

**Note:** This package is currently under heavy development.

## Usage
The basic usage is to launch the node together with a configuration (see, e.g.,
`config/parameters/viknes.yaml`) and an initial state.

An example launch file is found in `launch/example.launch`. It launches the
simulator together with a Line of Sight (LOS) node for waypoint following. In
addition a 3D-model is loaded using the `robot_state_publisher`-package. Rviz is
launched for visualization.
