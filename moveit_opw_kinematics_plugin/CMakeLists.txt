set(MOVEIT_LIB_NAME moveit_opw_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} SHARED
        src/moveit_opw_kinematics_plugin.cpp)

target_include_directories(${MOVEIT_LIB_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/opw_kinematics/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${MOVEIT_LIB_NAME}
        rclcpp
        random_numbers
        pluginlib
        moveit_core
        moveit_msgs
        orocos_kdl
        kdl_parser
        tf2_kdl
        EIGEN3
        )

# prevent pluginlib from using boost
target_compile_definitions(${MOVEIT_LIB_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${MOVEIT_LIB_NAME} PRIVATE "MOVEIT_OPW_KINEMATICS_PLUGIN_BUILDING_DLL")

install(DIRECTORY include/ DESTINATION include)