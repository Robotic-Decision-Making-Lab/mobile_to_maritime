cmake_minimum_required(VERSION 3.23)
project(message_transforms)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

add_library(message_transforms SHARED)
target_sources(
    message_transforms
    PRIVATE src/transforms.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/message_transforms/transforms.hpp
)
target_compile_features(message_transforms PUBLIC cxx_std_20)
target_link_libraries(
    message_transforms
    PUBLIC
        rclcpp::rclcpp
        ${tf2_geometry_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
)

generate_parameter_library(message_transforms_parameters
    src/message_transforms_parameters.yaml
)

add_executable(message_transforms_node)
target_sources(message_transforms_node PRIVATE src/message_transforms_node.cpp)
target_compile_features(message_transforms_node PUBLIC cxx_std_20)
target_link_libraries(
    message_transforms_node
    PRIVATE message_transforms_parameters rclcpp::rclcpp message_transforms
)

install(
    TARGETS message_transforms
    EXPORT export_message_transforms
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

install(
    TARGETS
        message_transforms_node
        message_transforms_parameters
        DESTINATION
        lib/message_transforms
)

install(DIRECTORY launch DESTINATION share/message_transforms)

ament_export_targets(export_message_transforms HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
