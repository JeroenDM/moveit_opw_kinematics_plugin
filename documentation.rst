OPW Kinematics Solver
=====================

Installation
------------

As the plugin is not yet released, you need to install it from source. This means adding the package to the **src** directory in the ros workspace.

  git clone https://github.com/JeroenDM/moveit_opw_kinematics_plugin


When using ROS kinetic, you should checkout the corresponding version of the package:

  cd moveit_opw_kinematics_plugin
  git checkout kinetic-devel

Configuration
-------------
First you need a MoveIt configurations for a compatebale robot
The configuration will be integrated in the MoveIt setup assistant in the future. For now, you can configure it manually by editing the **kinematics.yaml** file in the 
