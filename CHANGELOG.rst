^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_opw_kinematics_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2021-09-22)
------------------
* First release in noetic.
* update to new opw_kinematics API (with std::array)
* Add opw_kinematics dependency to package.xml and CMakeLists.txt.
* Remove opw_kinematics submodule.
* Change moveit_resources to more specific moveit_resources_fanuc_moveit_config
* Minor changes:
  - It's not a service based IK solver.
  - Parameters don't always come from 'kinematics.yaml'.
* Contributors: JeroenDM, gavanderhoorn.

0.2.1 (2020-05-13)
------------------
* First release in melodic.

0.1.1 (2020-05-13)
------------------
* First release in kinetic.