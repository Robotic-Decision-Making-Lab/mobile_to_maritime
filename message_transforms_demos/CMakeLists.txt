cmake_minimum_required(VERSION 3.8)
project(message_transforms_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/message_transforms_demos)

ament_package()
