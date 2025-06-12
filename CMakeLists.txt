if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    cmake_minimum_required(VERSION 2.8.12)
else()
    cmake_minimum_required(VERSION 3.10.2)
endif()

# 禁用测试
option(CATKIN_ENABLE_TESTING "Disable testing" OFF)

if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)
endif()
if(POLICY CMP0024)
    cmake_policy(SET CMP0024 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0024 NEW)
endif()

project(cobridge_tester)

if(CMAKE_VERSION VERSION_LESS "3.1")
    if(CMAKE_COMPILER_IS_GNUCXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
else()
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()


set(ROS1_DISTROS "indigo" "melodic" "noetic")
set(ROS2_DISTROS "foxy" "humble" "jazzy")

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    add_definitions(-DROS1_VERSION_INDIGO)
    set(ROS_DISTRIBUTION "indigo")
elseif("$ENV{ROS_DISTRO}" STREQUAL "melodic")
    add_definitions(-DROS1_VERSION_MELODIC)
    set(ROS_DISTRIBUTION "melodic")
elseif("$ENV{ROS_DISTRO}" STREQUAL "noetic")
    add_definitions(-DROS1_VERSION_NOETIC)
    set(ROS_DISTRIBUTION "noetic")
elseif("$ENV{ROS_DISTRO}" STREQUAL "foxy")
    add_definitions(-DROS2_VERSION_FOXY)
    set(ROS_DISTRIBUTION "foxy")
elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_definitions(-DROS2_VERSION_HUMBLE)
    set(ROS_DISTRIBUTION "humble")
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    add_definitions(-DROS2_VERSION_JAZZY)
    set(ROS_DISTRIBUTION "jazzy")
endif()


list(FIND ROS1_DISTROS ${ROS_DISTRIBUTION} ROS1_FOUND)
list(FIND ROS2_DISTROS ${ROS_DISTRIBUTION} ROS2_FOUND)

message(STATUS "ROS1_FOUND: " ${ROS1_FOUND})
message(STATUS "ROS2_FOUND: " ${ROS2_FOUND})

if(NOT ROS1_FOUND EQUAL -1)
    if(catkin_FOUND)
        message(STATUS "Building with catkin")
        set(ROS_BUILD_TYPE "catkin")

        find_package(catkin REQUIRED COMPONENTS roscpp std_srvs message_generation cv_bridge sensor_msgs)
        add_service_files(
                FILES
                Common.srv
                NewlineCRLF.srv
                NoRequest.srv
                NoResponse.srv
                Null.srv
        )
        generate_messages(DEPENDENCIES std_msgs)
        catkin_package(CATKIN_DEPENDS roscpp std_srvs message_runtime cv_bridge sensor_msgs)
        include_directories(${catkin_INCLUDE_DIRS})

        add_executable(tester_node ros1_tester/src/tester_node.cpp)
        add_dependencies(tester_node ${PROJECT_NAME}_generate_messages_cpp)
        target_link_libraries(tester_node ${catkin_LIBRARIES} )

    else()
        message(FATAL_ERROR "Could not find catkin")
    endif()
elseif(NOT ROS2_FOUND EQUAL -1)
    if(ament_cmake_FOUND)
        message(STATUS "Building with ament_cmake")
        set(ROS_BUILD_TYPE "ament_cmake")

        find_package(rclcpp REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(rosidl_default_generators REQUIRED)

        rosidl_generate_interfaces(${PROJECT_NAME}
                "srv/Common.srv"
                "srv/NewlineCRLF.srv"
                "srv/NoRequest.srv"
                "srv/NoResponse.srv"
                "srv/Null.srv"
                DEPENDENCIES std_msgs
        )

        add_executable(tester_node ros2_tester/src/tester_node.cpp)
        target_link_libraries(tester_node
          ${OpenCV_LIBRARIES}
        )

        rosidl_target_interfaces(tester_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
        
        ament_target_dependencies(tester_node
          rclcpp
          sensor_msgs
          std_msgs
          cv_bridge
        )

        ament_export_dependencies(rosidl_default_runtime)
        ament_export_dependencies(rclcpp)
        ament_export_dependencies(sensor_msgs)
        ament_export_dependencies(std_msgs)
        ament_export_dependencies(cv_bridge)

    else()
        message(FATAL_ERROR "Could not find ament_cmake")
    endif()
else()
    message(FATAL_ERROR "ROS_DISTRO environment variable not supported")
endif()

# install
if(ROS_BUILD_TYPE STREQUAL "catkin")
    message(STATUS "Install with catkin")
    install(TARGETS
            tester_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY
            ros1_tester/launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
    message(STATUS "Install with ament")
    install(TARGETS
            tester_node
            DESTINATION lib/${PROJECT_NAME}
    )

    install(DIRECTORY
            ros2_tester/launch/
            DESTINATION share/${PROJECT_NAME}/launch
    )
    ament_package()
endif()

