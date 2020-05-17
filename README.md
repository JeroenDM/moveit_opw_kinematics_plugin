# OPW Kinematics solver plugin for MoveIt

[![Build Status](https://travis-ci.org/JeroenDM/moveit_opw_kinematics_plugin.svg?branch=melodic-devel)](https://travis-ci.org/JeroenDM/moveit_opw_kinematics_plugin)

This is a [MoveIt!](https://moveit.ros.org/) plugin for [opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics). The package calculates closed form inverse kinematic solutions for typical industrial robots and was created by [Jmeyer1292](https://github.com/Jmeyer1292). For compatible robots, this plugin is a lightweight alternative for the [ikfast plugin](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_kinematics/ikfast_kinematics_plugin).

## Getting started
To use this plugin with another robot, clone this package inside your workspace:
```bash
cd catkin_ws/src/
git clone https://github.com/JeroenDM/moveit_opw_kinematics_plugin.git
```

Update the `config/kinematics.yaml` file in the [moveit configuration](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) for your robot.

```yaml
manipulator:
  kinematics_solver: moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin
```
In the near future this step should be handled by the MoveIt setup assistant. The solver also needs specific robot dimensions, which can be added in the robot support package. For example, we addedd the file `opw_kinematics_kr6r700sixx.yaml` to the in the kuka_kr6_support package of the [test resources](https://github.com/JeroenDM/kuka_test_resources) for this package.
```yaml
opw_kinematics_geometric_parameters:
  a1:  0.025
  a2: -0.035
  b:   0.000
  c1:  0.400
  c2:  0.315
  c3:  0.365
  c4:  0.080
opw_kinematics_joint_offsets: [0.0, -1.57079632679, 0, 0, 0, 0]
opw_kinematics_joint_sign_corrections: [-1, 1, 1, -1, 1, -1]
```
Hopefully, in the future most compatible robots will have this file already by default.

## Documentation
See [documentation.rst](documentation.rst).

## Credits
The general template is copied from the moveit [srv_kinematics_plugin](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_kinematics/srv_kinematics_plugin).
Some functions are directly copied from the package [descartes_opw_model](https://github.com/Jmeyer1292/descartes_opw_model). A big thank you to @simonschmeisser and @gavanderhoorn for discussing and reviewing pull requests and implementing some features.
