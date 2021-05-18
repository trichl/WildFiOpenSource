set(COMPONENT_REQUIRES "InterfaceI2C" "InterfaceSoftwareI2C" "InterfaceTiming" "HelperTime" "HelperBits" )
set(COMPONENT_SRCS "ModuleIMU_BMX160.cpp" "ModuleIMU_BMX160_WAKESTUB" )
set(COMPONENT_ADD_INCLUDEDIRS ".")
register_component()