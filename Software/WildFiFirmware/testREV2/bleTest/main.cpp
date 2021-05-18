#include "ESP32TrackerREV2.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "esp_bt.h"

ESP32TrackerREV2 device = ESP32TrackerREV2();

static void ble_app_set_addr() {
    ble_addr_t addr;
    int rc;
    rc = ble_hs_id_gen_rnd(1, &addr); // generate random address
    //assert(rc == 0);
    rc = ble_hs_id_set_rnd(addr.val); // use newly generated address
    //assert(rc == 0);   
}

static void ble_app_advertise() {
    struct ble_gap_adv_params adv_params;
    uint8_t uuid128[16];
    int rc;

    /* Arbitrarily set the UUID to a string of 0x11 bytes. */
    //memset(uuid128, 0x11, sizeof uuid128);
    for(uint8_t i=0; i<16; i++) {
        uuid128[i] = i;
    }

    /* Major version=2; minor version=10. */
    rc = ble_ibeacon_set_adv_data(uuid128, 2, 10, 0);
    //assert(rc == 0);

    /* Begin advertising. */
    adv_params = (struct ble_gap_adv_params){ 0 };

    rc = ble_gap_adv_start(BLE_OWN_ADDR_RANDOM, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL); // BLE_HS_FOREVER
    //assert(rc == 0);
}

static void ble_app_on_sync(void) {
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();

    /* Advertise indefinitely. */
    ble_app_advertise();
}

void ble_host_task(void *param) {
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
    nimble_port_freertos_deinit();
}

extern "C" void app_main() {
    while(1) {
        printf("AWAKE!\n");
        device.initPins();
        device.initNVS();

        //esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N0);
        //esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N0);
        esp_power_level_t p = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);
        printf("BLE POWER: %d\n", p);
        printf("BROWNOUT REG: %d\n", READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG));

        uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

        esp_nimble_hci_and_controller_init(); // BROWNOUT if <3.66V -> disabling brownout detector

        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector

        printf("HELLO?\n");

        nimble_port_init();
        ble_hs_cfg.sync_cb = ble_app_on_sync;

        printf("HELLO2?\n");

        nimble_port_freertos_init(ble_host_task); // in own thread
        device.delay(5000); // wait or do other stuff

        
        int ret = nimble_port_stop();
        if (ret == 0) {
            nimble_port_deinit();
            ret = esp_nimble_hci_and_controller_deinit();
            if (ret != ESP_OK) {
                printf("DEINIT FAIL\n");
            }
        }


        device.setCPUSpeed(ESP32_10MHZ);
        device.delay(3000);
        printf("BLE done, sleep\n");

        device.enableInternalRTCInterruptInDeepSleep(20);
        device.deepSleep();
    }
}