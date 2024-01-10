cmake_minimum_required(VERSION 3.0.2)
project(hada_steerbot_description)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    urdf
    xacro
)

catkin_package()

install(DIRECTORY launch urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    roslaunch_add_file_check(launch)
endif()