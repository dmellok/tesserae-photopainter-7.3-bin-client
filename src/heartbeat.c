#include "heartbeat.h"
#include "app_config.h"
#include "pmic.h"
#include "wifi_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_wifi.h"

static const char *TAG = "hb";

/* Battery telemetry: the PhotoPainter routes battery sense through the
 * AXP2101 fuel gauge, not an ADC divider. The PMIC returns mV and % in
 * one I2C transaction each -- no curve-fit ADC calibration, no
 * piecewise-linear Li-Po SoC table needed. */

static int current_rssi(void) {
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return 0;
    return ap.rssi;
}

/* Short-string mapping of the reset reason so the server can distinguish
 * a normal timer wake ("timer") from a brownout, panic, watchdog, or repeated
 * power-on -- the latter three signal real trouble on battery. */
static const char *wake_reason_str(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "poweron";
        case ESP_RST_EXT:       return "ext";
        case ESP_RST_SW:        return "sw";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "int_wdt";
        case ESP_RST_TASK_WDT:  return "task_wdt";
        case ESP_RST_WDT:       return "wdt";
        case ESP_RST_DEEPSLEEP: return "timer";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_SDIO:      return "sdio";
        case ESP_RST_USB:       return "usb";
        case ESP_RST_JTAG:      return "jtag";
        default:                return "unknown";
    }
}

void heartbeat_format_json(char *dst, size_t dst_sz,
                           int sleep_interval_s,
                           esp_reset_reason_t reset_reason,
                           time_t sleep_until) {
    if (!dst || dst_sz == 0) return;

    int mv  = pmic_battery_mv();
    int pct = pmic_battery_pct();
    if (pct < 0) pct = 0;   /* fuel gauge unsure -- report 0, server treats 0+0mV as unknown */
    int rssi = current_rssi();
    char ip[16] = {0};
    wifi_manager_get_sta_ip(ip, sizeof(ip));
    const char *wake = wake_reason_str(reset_reason);

    ESP_LOGI(TAG, "battery=%d mV (%d%%), rssi=%d dBm, ip=%s, fw=%s, "
                  "sleep=%ds, wake=%s, sleep_until=%lld",
             mv, pct, rssi, ip, FW_VERSION, sleep_interval_s, wake,
             (long long) sleep_until);

    /* Base payload: all unconditional fields. next_sleep_s mirrors
     * sleep_interval_s for this firmware (we always sleep for the
     * configured interval); the two keys exist because Tesserae's
     * smart-sync wire contract distinguishes "configured cadence" from
     * "actual upcoming sleep" -- firmwares that dynamically extend
     * sleep on low battery could publish different values. */
    int n = snprintf(dst, dst_sz,
             "{\"battery_mv\":%d,\"battery_pct\":%d,\"rssi\":%d,"
             "\"ip\":\"%s\",\"fw_version\":\"%s\","
             "\"kind\":\"%s\",\"panel_w\":%d,\"panel_h\":%d,"
             "\"sleep_interval_s\":%d,\"next_sleep_s\":%d,"
             "\"wake_reason\":\"%s\"",
             mv, pct, rssi, ip, FW_VERSION, DEVICE_KIND,
             EPD_WIDTH, EPD_HEIGHT, sleep_interval_s, sleep_interval_s,
             wake);

    if (n < 0 || (size_t) n >= dst_sz) return;   /* truncated; leave as-is */

    /* sleep_until is optional -- omit when NTP isn't synced so the
     * server falls back to its tolerance-window math instead of seeing
     * 0/garbage as a wall-clock timestamp. */
    if (sleep_until > 0) {
        int m = snprintf(dst + n, dst_sz - n,
                         ",\"sleep_until\":%lld", (long long) sleep_until);
        if (m > 0 && (size_t)(n + m) < dst_sz) n += m;
    }

    if ((size_t) n < dst_sz - 1) {
        dst[n]     = '}';
        dst[n + 1] = '\0';
    } else if (dst_sz > 0) {
        dst[dst_sz - 1] = '\0';
    }
}
