^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_opw_kinematics_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2022-01-05)
------------------
* Update status badge noetic-devel (default) branch
* Run melodic ci job on ubuntu 18.04
* Sync melodic changes to noetic
  - fix bug introduced when cherry-picking commits
  - minor formatting improvement
  - replace logstring with variable LOGNAME
  - print out odd posture used in self-test on error
  - Print all deviations and check zero pose as well
* Update Jeroen's email address
* Add github actions
  - Add roscpp as CATKIN_DEPENDS
  - Use the .ci.rosinstall to install dependencies
  - Move rosinstall for testing to typical place for industrial-ci
  - Update ci badges to GHA
  - Remove travis
* Contributors: JeroenDM, Simon Schmeisser

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