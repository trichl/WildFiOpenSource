set(COMPONENT_REQUIRES "InterfaceI2C" "InterfaceTiming")
set(COMPONENT_SRCS "ModuleBARO_BSEC_BME680.cpp" "bme680.c" "bsec.cpp")
set(COMPONENT_ADD_INCLUDEDIRS ".")
register_component()