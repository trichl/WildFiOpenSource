#ifndef config_h
#define config_h

#include "PlatformWildFiTagREV6.h"
#include "passwords.h"
#include "definesForProximity.h"

/** IMPORTANT: SELECT PROJECT SPECIFIC CONFIGURATION HERE */
#include "configDevelopment.h"

#define WILDFI_SOFTWARE_VERSION                         42

/** Functions */
void printConfigurationHash(tag_config_t *c);
bool configIsPlausible(tag_config_t *config, uint8_t *errorIdIn);
void printConfiguration(tag_config_t *c);
bool checkIfReconfigNeeded(WildFiTagREV6 *device, tag_config_t *currentConfig, uint8_t *nearestGatewayConfiguration, uint8_t nearestGatewayCommand, uint8_t nearestGatewayRssi, bool *changedSomething, bool debug);
bool readConfigFromNVS(WildFiTagREV6 *device, tag_config_t *config, bool printConfig);

#endif