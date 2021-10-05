#include "PlatformWildFiTagREV6.h"
//#include "ESPNtpClient.h"

#include "lwip/init.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/dns.h"
#include "sys/time.h"
#include "time.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"

WildFiTagREV6 device = WildFiTagREV6();

RTC_DATA_ATTR uint64_t bootCnt = 0;


udp_pcb* udp;
bool isConnected = false;
#define DEFAULT_NTP_PORT            123 ///< @brief Default local udp port. Select a different one if neccesary (usually not needed)
#define DEFAULT_NTP_INTERVAL        1800 ///< @brief Default sync interval 30 minutes
#define DEFAULT_NTP_SHORTINTERVAL   15 ///< @brief Sync interval when sync has not been achieved. 15 seconds
#define DEFAULT_NTP_TIMEOUT         5000 ///< @brief Default NTP timeout ms

uint32_t shortInterval = DEFAULT_NTP_SHORTINTERVAL * 1000;  ///< @brief Interval to set periodic time sync until first synchronization.
uint32_t longInterval = DEFAULT_NTP_INTERVAL * 1000;        ///< @brief Interval to set periodic time sync
uint32_t actualInterval = DEFAULT_NTP_SHORTINTERVAL * 1000; ///< @brief Currently selected interval

void receviceUDPPacket(void* arg, struct udp_pcb* pcb, struct pbuf* p, const ip_addr_t* addr, u16_t port) {
    // TODO
}

bool highAccuracyNTP() {
    udp = udp_new ();
    if (!udp){
        printf("Failed to create NTP socket\n");
        return false;
    }
    printf("NTP socket created\n");

    if(device.connectedToWiFi() != WIFI_CONNECT_SUCCESS) {
        printf("Not connected to WiFi\n");
        return false;
    }

    ip_addr_t localAddress;
    //esp_ip4_addr_t wifiIp = device.getWiFiIP();
    //localAddress.u_addr.ip4.addr = wifiIp.addr; // TODO

    localAddress.type = IPADDR_TYPE_V4;
    printf("Bind UDP port %d to %08X", DEFAULT_NTP_PORT, localAddress.u_addr.ip4.addr);

    err_t result = udp_bind(udp, &localAddress, DEFAULT_NTP_PORT);
    if(result) {
        printf("Failed to bind to NTP port. %d: %s", result, lwip_strerr(result));
        if(udp) {
            udp_disconnect (udp);
            udp_remove (udp);
            udp = NULL;
        }
        isConnected = false;
        actualInterval = shortInterval;
        return false;
    } else {
        isConnected = true;
    }
    udp_recv(udp, &receviceUDPPacket, NULL); // last parameter was "this"
    return true;
}

extern "C" void app_main() {
    while(1) {
        if(bootCnt == 0) {
            printf("HELLO WORLD\n");

        }
        else {

        }
        bootCnt++;
        device.enableInternalTimerInterruptInDeepSleep(3);
        device.deepSleep();
    }
}
