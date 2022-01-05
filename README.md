# opw_kinematics_plugin

[![Build status Ubuntu Bionic](https://github.com/JeroenDM/moveit_opw_kinematics_plugin/actions/workflows/ci_bionic.yml/badge.svg?branch=noetic-devel)](https://github.com/JeroenDM/moveit_opw_kinematics_plugin/actions/workflows/ci_bionic.yml)
[![Build status Ubuntu Focal](https://github.com/JeroenDM/moveit_opw_kinematics_plugin/actions/workflows/ci_focal.yml/badge.svg?branch=noetic-devel)](https://github.com/JeroenDM/moveit_opw_kinematics_plugin/actions/workflows/ci_focal.yml)

An attempt at writing a [MoveIt!](https://moveit.ros.org/) plugin for [opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics). The package calculates closed form inverse kinematic solutions for typical industrial robots and was created by [Jmeyer1292](https://github.com/Jmeyer1292). A MoveIt! plugin for this package could be an alternative for the [ikfast plugin](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_kinematics/ikfast_kinematics_plugin).

See [issues](https://github.com/JeroenDM/moveit_opw_kinematics_plugin/issues) the follow the implementation process. (Where I mostly have conversations with myself.)

The general template is copied from the moveit [srv_kinematics_plugin](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_kinematics/srv_kinematics_plugin).
Some functions are directly copied from the package [descartes_opw_model](https://github.com/Jmeyer1292/descartes_opw_model).

To use this plugin with another robot, clone this package inside your workspace:
```bash
cd catkin_ws/src/
git clone --recursive https://github.com/JeroenDM/moveit_opw_kinematics_plugin.git
```

And also add a [moveit configuration](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) for a compatible robot. You have to update the config/kinematics.yaml file. It looks like this for a Kuka kr6r700:

```yaml
manipulator:
  kinematics_solver: moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin
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
