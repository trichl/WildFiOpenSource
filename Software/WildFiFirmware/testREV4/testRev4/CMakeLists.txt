# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/testREV4/testRev4/*.*)

idf_component_register(SRCS ${app_sources})
