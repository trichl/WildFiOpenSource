set(COMPONENT_REQUIRES "HelperBits" "InterfaceTiming" "InterfaceI2C" "InterfaceSoftwareI2C" "ModuleBARO_BME680" "ModuleFLASH_MT29" "ModuleIMU_BMX160" "ModuleRTC_RV8803C7" "esp_adc_cal" "spiffs" "nvs_flash" "esp_http_client" "esp-tls" "bt")
set(COMPONENT_SRCS "PlatformWildFiTagREV6.cpp" "esp32-hal-cpu.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")
register_component()