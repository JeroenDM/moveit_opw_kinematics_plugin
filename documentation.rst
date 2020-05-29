OPW Kinematics Solver
=====================

This solver provides analytical inverse kinematics for a specific type of industrial robots. If it works for your robot, it is significantly faster that the default numerical solver in MoveIt.

Installation
------------

Install the package through the normal process:

  sudo apt install ros-melodic-moveit-opw-kinematics-plugin

where you can replace :literal:`melodic` with :literal:`kinetic` if you're using the kinetic ROS distro. Those are the only two suported versions, although the melodic branch will probably aslo work in newer versions.

To install the package from source, add the following git repository to the **src** directory in the ros workspace.

  git clone https://github.com/JeroenDM/moveit_opw_kinematics_plugin -b melodic-devel


When using ROS kinetic, you should checkout the corresponding version of the package:

  cd moveit_opw_kinematics_plugin
  git checkout kinetic-devel

Configuration
-------------
First you need a MoveIt configurations for a compatible robot.
The configuration will be integrated in the MoveIt setup assistant in the future. For now, you can configure it manually by editing the **kinematics.yaml** file in the MoveIt configuration package of youre robot (e.g. :literal:`myrobot_moveit_config`). This file should contain:

  manipulator:
    kinematics_solver: moveit_opw_kinematics_plugin/MoveItOPWKinematicsPlugin

In addition, the robot support package (e.g. :literal:`myrobot_support`) should have a yaml file with specific dimensions of the robot that the solver needs to calculate the inverse kinematics. If you're lucky, the robot support package already contains a file that looks like this example from the kuka_test_resources_ repository.
:literal:`opw_kinematics_kr6r700sixx.yaml`

.. code-block:: yaml

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

If you're less lucky, you should find out these paramters by comparing the datasheet with the following figure from the readme of the opw_kinematics_ repository and included below:

.. image:: geometric_parameters.png

Usage
-----

TODO

- Mostly used indirectly through MoveIt interfaces.
- What functions should I use when calling the plugin directly.


Background
----------

This solver provides analytical inverse kinematics for a specific type of industrial robots as described in the paper_ :literal:`An Analytical Solution of the Inverse Kinematics Problem of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist` by Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur. This plugin uses an implementation of the algorithm by Jonathan Meyer, called opw_kinematics_.


.. _paper: http://modular-machines.at/images/Modular-Machines/pub/BraAngHof14.pdf
.. _kuka_test_resources: https://github.com/JeroenDM/kuka_test_resources
.. _opw_kinematics: https://github.com/Jmeyer1292/opw_kinematics