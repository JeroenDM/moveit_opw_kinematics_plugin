# opw_kinematics_plugin
An attempt at writing a MoveIt! plugin for [opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics). See [issues](https://github.com/JeroenDM/moveit_opw_kinematics_plugin/issues) the follow the implementation process. (Where I mostly have conversations with myself.)

To use this plugin with another robot, clone this package inside your workspace:
```bash
cd catkin_ws/src/
git clone https://github.com/JeroenDM/moveit_opw_kinematics_plugin.git
```

And also add a [moveit configuration](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) for a compatible robot. You have to update the config/kinematics.yaml file. It looks like this for a Kuka kr6r700:

```yaml
manipulator:
  kinematics_solver: moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin
  kinematics_solver_geometric_parameters:
    a1:  0.025
    a2: -0.035
    b:   0.000
    c1:  0.400
    c2:  0.315
    c3:  0.365
    c4:  0.080
  kinematics_solver_joint_offsets: [0.0, -1.57079632679, 0, 0, 0, 0]
  kinematics_solver_joint_sign_corrections: [-1, 1, 1, -1, 1, -1]
```
