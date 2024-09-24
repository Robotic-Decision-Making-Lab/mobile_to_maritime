cmake_minimum_required(VERSION 3.8)
project(message_transforms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    geometry_msgs
    nav_msgs
    rclcpp_components
    tf2_geometry_msgs
)

foreach(pkg IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${pkg} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(message_transforms_parameters
    src/message_transforms_parameters.yaml
)

add_library(message_transforms_library SHARED)

target_sources(
    message_transforms_library
    PRIVATE src/message_transforms_node.cpp src/transforms.cpp
)
target_include_directories(
    message_transforms_library
    PUBLIC
        $<INSTALL_INTERFACE:include/message_transforms>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(
    message_transforms_library
    PRIVATE message_transforms_parameters
)
ament_target_dependencies(message_transforms_library PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(message_transforms_library PUBLIC cxx_std_20)

rclcpp_components_register_node(message_transforms_library
    PLUGIN "m2m::MessageTransforms"
    EXECUTABLE message_transforms
)

install(DIRECTORY include/ DESTINATION include/message_transforms)

install(
    TARGETS message_transforms_library message_transforms_parameters
    EXPORT export_message_transforms_library
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY launch DESTINATION share/message_transforms)

ament_export_targets(export_message_transforms_library HAS_LIBRARY_TARGET)
ament_export_dependencies($THIS_PACKAGE_INCLUDE_DEPENDS)

ament_package()
