# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES "PlatformWildFiTagREV6" "ModuleGPS_L70_REV6" )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" "config.cpp" ) # set main file to use
set(COMPONENT_ADD_INCLUDEDIRS )

#add_subdirectory(WildFiLibrary)
#target_link_libraries(${COMPONENT_LIB} PUBLIC WildFiLibrary)

register_component()
