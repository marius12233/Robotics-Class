# unisa_trajectory_planner

This package provides a planning node loading a path from file and generating a joint-space trajectory. It also allows the user to verify that joint limits are respected by publishing trajectories to `rqt_multiplot`.

Trajectory files should be in `.traj` format and be placed in the data folder of this package. One way to generate `.traj` files is through the MATLAB function `export_ros_workspace_path`. An example trajectory is provided in the `/data` folder of this package.

Trajectory files are loaded by using the `WorkspaceTrajectory` class of the `moveit_dp_redundancy_resolution` package, which is, therefore, a dependency. Also, this package takes a dependency on `smartsix_moveit_config`, defining the planning information for the Comau SmartSix robot, used as an example.

## How to run the planner

Running the planner needs the `robot_description` parameter be loaded on the parameter server. You can do it for the Comau SmartSix by executing

```bash
roslaunch smartsix_moveit_config demo.launch
```

Run the planner with

```bash
roslaunch unisa_trajectory_planner planner.launch
```

This will also launch `rqt_multiplot`. The first time `rqt_multiplot` is launched, the configuration file `rqt_multiplot.xml` (find it in the `/config` folder of this package) can be loaded to automatically configure the plots.
