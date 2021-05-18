#include "InterfaceTiming.h"

void Timing::delay(uint16_t d) {
	vTaskDelay(d / portTICK_PERIOD_MS);
}

uint64_t Timing::millis() {
    return (uint64_t) (esp_timer_get_time() / 1000ULL);
}
