hoap3\_write
============

This package contains a set of scripts useful to generate joint-space trajectories that
follow a SVG shape.

It requires MoveIt for the robot inverse kinematics, and softMotion for the SVG2traj 
conversion.

Available tools
---------------

- `scripts/place_paper.py`: displays an interactive marker in RViz that is
  shaped as an A4 paper sheet. Used to visually place the SVG trajectory in
  space.

- `scripts/publish_traj`: takes a SVG file as input and publishes it as a ROS
  topic (`/write_traj`). Can also optionally display it in RViz.

- `scripts/sample_reachable.py`: sample random points around the robot
  end-effector to see if they are reachable by IK.

- `scripts/write.py`: reads a cartesian trajectory from ROS topic `write_traj`
  and generate the corresponding joint-space trajectory.


