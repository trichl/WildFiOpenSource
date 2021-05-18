# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/testREV6/proximityDetectionEspNow/*.*)

idf_component_register(SRCS ${app_sources})


#IF ULP SHALL BE USED (directory ulp under root not examples):
#idf_component_register(SRCS "main.cpp"
#                    INCLUDE_DIRS ""
#                    REQUIRES soc nvs_flash ulp driver)
# 1. The ULP app name must be unique (if multiple components use ULP).
#set(ulp_app_name ulp_main)
# 2. Specify all assembly source files. Files should be placed into a separate directory (in this case, ulp/), which should not be added to COMPONENT_SRCS.
#set(ulp_s_sources "../../ulp/test.S")
# 3. List all the component source files which include automatically generated ULP export file, ${ulp_app_name}.h:
#set(ulp_exp_dep_srcs "main.cpp")
# 4. Call function to build ULP binary and embed in project using the argument values above.
#ulp_embed_binary(${ulp_app_name} ${ulp_s_sources} ${ulp_exp_dep_srcs})
