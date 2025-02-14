cmake_minimum_required(VERSION 3.8)
project(rc_planning)

set(CXX_STANDARD_REQUIRED 17)
set(CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -std=gnu++17 -Wno-strict-aliasing -Wno-unused-but-set-variable -Wno-sign-compare -Wno-unused-variable -fopenmp -O3 -ftree-vectorize -funroll-loops -fPIE)
endif()

string(ASCII 27 Esc)
set(RED "${Esc}[1;31m")
set(YELLOW "${Esc}[1;33m")
set(NONE "${Esc}[m")
set(BLUE "${Esc}[1;34m")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message("${BLUE}select arch with ${CMAKE_SYSTEM_PROCESSOR}")
    set(CMAKE_PREFIX_PATH "/opt/ros/humble/share;/usr/lib/x86_64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    message("${BLUE}select arch with ${CMAKE_SYSTEM_PROCESSOR}")
    set(CMAKE_PREFIX_PATH "/opt/ros/humble/share;/usr/lib/arm-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch")
    message("${BLUE}select arch with ${CMAKE_SYSTEM_PROCESSOR}")
    set(CMAKE_PREFIX_PATH "/opt/ros/humble/share;/usr/lib/aarch64-linux-gnu")
else()
    message("${BLUE}unknown arch with ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS True)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    add_definitions(-DEIGEN_USE_BLAS)
    find_package(BLAS REQUIRED)
    set(BLAS_LIBRARIES "/usr/lib/x86_64-linux-gnu/openblas-openmp/libopenblas.so;-lm;-ldl")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bondcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(class_loader REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_publisher REQUIRED)
find_package(magic_enum REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters features keypoints sample_consensus)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(Qhull REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(ros2_socketcan_msgs REQUIRED)

add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/include/eigen3
    ${pcl_conversions_INCLUDE_DIRS}
    ${rclcpp_lifecycle_INCLUDE_DIRS}
)

set(dependencies
    rclcpp
    rcutils
    std_msgs
    sensor_msgs
    geometry_msgs
    OpenCV
    cv_bridge
    image_transport
    nav2_msgs
    nav2_util
    nav_msgs
    pluginlib
    visualization_msgs
    tf2_ros
    tf2
    std_srvs
    rclcpp_components
    action_tutorials_interfaces
    builtin_interfaces
    rosidl_runtime_cpp
    camera_info_manager
    class_loader
    image_geometry
    image_publisher
    magic_enum
    tf2_eigen
    tf2_msgs
    tf2_sensor_msgs
    PCL
    pcl_ros
    pcl_conversions
    rclcpp_lifecycle
    pcl_msgs
    bondcpp
    yaml-cpp
    ros2_socketcan_msgs
    ros2_socketcan
)

add_executable(rc_planning
    src/robot_planning_node.cpp
)
ament_target_dependencies(rc_planning ${dependencies})
target_link_libraries(rc_planning
    ${OpenCV_LIBS}
    ${Qhull_LIBS}
    ${YAML_CPP_LIBRARIES}
    Eigen3::Eigen
    magic_enum::magic_enum
)

install(TARGETS
    rc_planning
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config "include"
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
