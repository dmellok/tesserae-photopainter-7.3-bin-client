/*
 * tesserae-photopainter-7.3-bin-client -- battery-powered MQTT-driven
 * e-paper client for the Waveshare ESP32-S3 PhotoPainter (7.3" Spectra
 * E6, 800 x 480). Subscribes to a retained frame URL, downloads the
 * panel-native 192000-byte 4bpp .bin, paints it, and goes back to deep
 * sleep.
 *
 * Lifecycle of one wake:
 *
 *   boot
 *     -> pmic_init() / wifi_manager_init()
 *     -> start button-watch task (polls GPIO0 the whole wake)
 *     -> wifi creds in NVS?            no  -> captive portal -> reboot
 *     -> connect STA                   fail-> captive portal -> reboot
 *     -> BOOT long-press detected?     yes -> settings server -> reboot
 *     -> ensure NTP synced (3 s timeout; cached across deep sleeps)
 *     -> grab retained MQTT job (URL only -- heartbeat publishes later)
 *     -> need paint? (URL changed AND no hash match, OR BOOT tap)
 *           yes -> fetch + decode + drop wifi + paint
 *                  + reconnect wifi + publish heartbeat
 *           no  -> publish heartbeat on the existing MQTT session
 *     -> deep sleep for sleep_interval_s
 *
 * The heartbeat publishes at the END of every wake so its sleep_until
 * is wall-clock-accurate for Tesserae's smart-sync scheduler
 * (https://github.com/dmellok/tesserae/issues/10) -- the server uses
 * it to JIT-render the next frame so it lands in the retained slot
 * right before the device wakes.
 *
 * The BOOT button (GPIO0) gives the user two runtime actions:
 *   tap   -> force a re-paint even if the URL hash matches NVS
 *   hold  -> open the LAN settings editor instead of the paint cycle
 * Both are detected concurrently with the wake cycle via a background
 * task, so a press registered at any point during the few-second wake
 * takes effect at the next decision point.
 */

#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"
#include "nvs.h"

#include "app_config.h"
#include "epd_driver.h"
#include "heartbeat.h"
#include "image_decoder.h"
#include "image_fetcher.h"
#include "mqtt_handler.h"
#include "pmic.h"
#include "provisioning.h"
#include "splash.h"
#include "wifi_manager.h"

static const char *TAG = "main";

/* ---------- BOOT button watcher ---------- */

#define BIT_BTN_TAP    BIT0   /* short press completed (released)         */
#define BIT_BTN_HOLD   BIT1   /* press held past BTN_HOLD_MIN_MS          */

static EventGroupHandle_t s_btn_events;

/* Polls GPIO0 at 50 Hz from a low-priority task started early in
 * app_main(). Tracks press start time, sets BIT_BTN_HOLD as soon as the
 * threshold is crossed (one-shot per press) and BIT_BTN_TAP on release
 * when total press duration was under BTN_TAP_MAX_MS. Runs for the whole
 * wake -- never exits -- because we deep-sleep before the cycle ends. */
static void button_watch_task(void *arg) {
    (void) arg;
    gpio_config_t in = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BTN_PIN_BOOT),
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&in);
    /* Give the internal pull-up a moment to overcome line capacitance
     * before the very first read. */
    vTaskDelay(pdMS_TO_TICKS(20));
    /* Log the resting level so we can spot a stuck-low pin (which would
     * masquerade as a permanent press and lock the device into settings
     * mode on every boot). Expected: 1 (HIGH, pulled up, not pressed). */
    ESP_LOGI(TAG, "BOOT (GPIO%d) resting level=%d  (expect 1 = released)",
             BTN_PIN_BOOT, gpio_get_level(BTN_PIN_BOOT));

    bool pressed = false;
    bool hold_logged = false;
    int64_t press_start_us = 0;

    while (1) {
        bool now = (gpio_get_level(BTN_PIN_BOOT) == 0);
        int64_t t = esp_timer_get_time();

        if (now && !pressed) {
            pressed = true;
            press_start_us = t;
            hold_logged = false;
        } else if (now && pressed) {
            int64_t held_ms = (t - press_start_us) / 1000;
            if (!hold_logged && held_ms >= BTN_HOLD_MIN_MS) {
                xEventGroupSetBits(s_btn_events, BIT_BTN_HOLD);
                ESP_LOGI(TAG, "BOOT long press (%lld ms) -> settings mode",
                         (long long) held_ms);
                hold_logged = true;
            }
        } else if (!now && pressed) {
            int64_t held_ms = (t - press_start_us) / 1000;
            if (!hold_logged && held_ms < BTN_TAP_MAX_MS) {
                xEventGroupSetBits(s_btn_events, BIT_BTN_TAP);
                ESP_LOGI(TAG, "BOOT tap (%lld ms) -> force refresh",
                         (long long) held_ms);
            }
            pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static bool button_tap_seen(void) {
    return (xEventGroupGetBits(s_btn_events) & BIT_BTN_TAP) != 0;
}
static bool button_hold_seen(void) {
    return (xEventGroupGetBits(s_btn_events) & BIT_BTN_HOLD) != 0;
}

/* ---------- "should I bother re-rendering?" ---------- */

static void sha256_hex(const char *in, char out_hex[65]) {
    uint8_t digest[32];
    mbedtls_sha256((const unsigned char *)in, strlen(in), digest, 0);
    for (int i = 0; i < 32; i++) {
        static const char H[] = "0123456789abcdef";
        out_hex[i*2]   = H[(digest[i] >> 4) & 0xF];
        out_hex[i*2+1] = H[digest[i] & 0xF];
    }
    out_hex[64] = '\0';
}

static bool hash_matches_stored(const char *hash) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS_STATE, NVS_READONLY, &h) != ESP_OK) return false;
    char prev[65] = {0};
    size_t len = sizeof(prev);
    esp_err_t err = nvs_get_str(h, NVS_KEY_LAST_HASH, prev, &len);
    nvs_close(h);
    return err == ESP_OK && strcmp(prev, hash) == 0;
}

static void store_hash(const char *hash) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS_STATE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, NVS_KEY_LAST_HASH, hash);
    nvs_commit(h);
    nvs_close(h);
}

/* Read the MQTT-configured sleep interval from NVS, falling back to the
 * compile-time SLEEP_INTERVAL_S default. Clamped defensively in case
 * something wrote a bad value before the bounds check existed. */
static int load_sleep_interval_s(void) {
    int32_t v = 0;
    nvs_handle_t h;
    if (nvs_open(NVS_NS_STATE, NVS_READONLY, &h) == ESP_OK) {
        esp_err_t err = nvs_get_i32(h, NVS_KEY_SLEEP_S, &v);
        nvs_close(h);
        if (err == ESP_OK &&
            v >= SLEEP_INTERVAL_MIN_S &&
            v <= SLEEP_INTERVAL_MAX_S) {
            return v;
        }
    }
    return SLEEP_INTERVAL_S;
}

/* ---------- deep sleep ---------- */

static void sleep_forever_or_until_timer(void) {
    /* Decide between deep sleep (battery) and short-delay restart loop (dev):
     *   DEV_DISABLE_SLEEP defined -> always loop (dev override)
     *   DEV_FORCE_SLEEP   defined -> always deep-sleep, even on USB host
     *   otherwise -> auto-detect: USB host (laptop / SOF-emitter) loops, a
     *                bare USB charger / power bank does not emit SOFs. */
#if defined(DEV_DISABLE_SLEEP) && defined(DEV_FORCE_SLEEP)
#  error "DEV_DISABLE_SLEEP and DEV_FORCE_SLEEP are mutually exclusive"
#endif

    bool loop = false;
    const char *reason = NULL;

#ifdef DEV_DISABLE_SLEEP
    loop = true;
    reason = "DEV_DISABLE_SLEEP";
#elif defined(DEV_FORCE_SLEEP)
    /* Skip the USB-host check entirely; behave as if on battery. */
#else
    if (usb_serial_jtag_is_connected()) {
        loop = true;
        reason = "USB host detected";
    }
#endif

    if (loop) {
        ESP_LOGI(TAG, "%s: software restart in %d s", reason, DEV_LOOP_INTERVAL_S);
        vTaskDelay(pdMS_TO_TICKS(DEV_LOOP_INTERVAL_S * 1000));
        esp_restart();
    }

    int interval = load_sleep_interval_s();
    ESP_LOGI(TAG, "on battery; deep sleep for %d s%s",
             interval,
             (interval == SLEEP_INTERVAL_S) ? " (default)" : " (set via mqtt)");
    /* epd_sleep() already dropped the panel rails via the PMIC; no
     * extra cleanup needed before going down. */
    esp_sleep_enable_timer_wakeup((uint64_t)interval * 1000000ULL);
    esp_deep_sleep_start();
    /* not reached */
}

/* ---------- app ---------- */

static void run_provisioning_then_reboot(void) {
    ESP_LOGW(TAG, "no usable wifi creds; painting portal splash + captive portal");
    /* Paint logo + WPA QR before bringing up the AP so the user can scan
     * to join Tesserae-Setup instead of typing the SSID. ~30 s panel
     * refresh runs concurrently with the AP/HTTPD/DNS bringup. */
    splash_show_portal();
    esp_err_t err = provisioning_run_blocking();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "creds saved; rebooting to use them");
    } else {
        ESP_LOGW(TAG, "portal timed out; sleeping and trying again later");
    }
    /* Either way: reboot so STA path starts fresh next time. */
    esp_restart();
}

/* ---------- wall-clock time + heartbeat ---------- */

/* Reasonable-epoch window for time(NULL):
 *   MIN = 2023-11-14  -- catches the ESP-IDF default 2016 epoch when
 *                        NTP has never synced (or before it does)
 *   MAX = 2039-09-13  -- catches a misconfigured SNTP server returning
 *                        a wild far-future date
 * Outside this window we treat the clock as bogus and omit sleep_until
 * rather than ship the bad value. */
#define EPOCH_REASONABLE_MIN  1700000000LL
#define EPOCH_REASONABLE_MAX  2200000000LL

/* NTP sync helper.
 *
 * On most ESP32-S3 boards the RTC slow clock keeps running through
 * deep sleep and the POSIX time advances naturally, so a cached-time
 * skip would be cheap. Unfortunately on the PhotoPainter this does
 * NOT hold: empirically `time(NULL)` reads ~26 s behind wall-clock on
 * every timer-wake from deep sleep, almost certainly because the
 * AXP2101 PMIC sleep state interferes with the SoC's RTC time-of-day
 * maintenance. The bug is invisible until you cross-check the
 * heartbeat's sleep_until against the broker-receive timestamp --
 * Tesserae's smart-sync scheduler then marks the device "untrusted"
 * because predicted-wake-time disagrees with observed-wake-time by
 * tens of seconds.
 *
 * So this firmware does a fresh SNTP exchange on EVERY wake, not just
 * on cold boot. The cost is +1-5 s of WiFi-on time per wake (~0.02-
 * 0.11 mAh extra) but every published sleep_until is wall-clock-
 * accurate. `force_resync` is retained as a parameter for symmetry
 * with the sibling firmware's API and for forward compatibility -- a
 * future PhotoPainter board with a 32 K crystal could re-enable the
 * cached-time skip by checking !force_resync.
 *
 * 5 s timeout: 3 s was empirically too tight on residential WiFi for
 * the cold-boot first lookup (DNS + NTP round-trip). 5 s reliably
 * succeeds. */
static void ensure_time_synced(int wait_ms, bool force_resync) {
    (void) force_resync;   /* always re-syncs on this hardware */
    time_t cached = time(NULL);
    ESP_LOGI(TAG, "ntp sync (cached epoch=%lld, force=%d)",
             (long long) cached, (int) force_resync);

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sntp init failed: %s; sleep_until will be omitted",
                 esp_err_to_name(err));
        return;
    }

    err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(wait_ms));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ntp synced; epoch=%lld", (long long) time(NULL));
    } else {
        ESP_LOGW(TAG, "ntp sync timeout (%dms); sleep_until will be omitted",
                 wait_ms);
    }
    esp_netif_sntp_deinit();
}

/* Format + publish the heartbeat right before sleep so sleep_until is
 * wall-clock-accurate (Tesserae's smart-sync wire contract). Caller
 * must have WiFi up; this function brings MQTT up one-shot, publishes
 * retained QoS 1, waits for the PUBACK, tears MQTT down. Failures are
 * logged and swallowed -- a missed heartbeat just means the server's
 * next prediction falls back to its tolerance-window math for one
 * cycle.
 *
 * sleep_until guards (any of these tripping omits the key):
 *   - clock not in [2023-11-14, 2039-09-13] -- NTP not synced or
 *     server returned garbage
 *   - cross-check delta = sleep_until - now outside [sleep_s-5,
 *     sleep_s+5] -- tautological with `now + sleep_s` today, but a
 *     future refactor that computes sleep_until via a different path
 *     would be caught here, and the log warning is the easiest signal
 *     to discover such a bug. */
static void publish_heartbeat(int sleep_s, esp_reset_reason_t reset_reason) {
    char hb[320];
    time_t now = time(NULL);
    time_t sleep_until = 0;

    if (now > EPOCH_REASONABLE_MIN && now < EPOCH_REASONABLE_MAX) {
        time_t candidate = now + sleep_s;
        long delta = (long)(candidate - now);
        if (delta < (long) sleep_s - 5 || delta > (long) sleep_s + 5) {
            ESP_LOGW(TAG,
                "sleep_until cross-check failed: delta=%ld, sleep_s=%d; omitting",
                delta, sleep_s);
        } else {
            sleep_until = candidate;
        }
    } else if (now != 0) {
        ESP_LOGW(TAG,
            "epoch=%lld outside sane window [%lld, %lld]; omitting sleep_until",
            (long long) now,
            (long long) EPOCH_REASONABLE_MIN,
            (long long) EPOCH_REASONABLE_MAX);
    }

    heartbeat_format_json(hb, sizeof hb, sleep_s, reset_reason, sleep_until);

    esp_err_t err = mqtt_publish_status(hb);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "heartbeat publish failed: %s", esp_err_to_name(err));
    }
}

/* Show the Tesserae logo splash on a true cold boot (power-on / RESET).
 * Skip it on timer-wake (production sleep cycle) AND on software restart
 * (DEV_DISABLE_SLEEP / DEV_FORCE_SLEEP loop iterations) so we don't burn
 * 25 s of panel refresh on every quick test cycle.
 *
 * If we'll be going straight to the captive portal anyway (no creds),
 * skip the logo splash -- run_provisioning_then_reboot() paints the
 * portal splash instead, avoiding a wasted second ~30 s refresh. */
static void maybe_show_splash(esp_reset_reason_t reset_reason, bool has_creds) {
    if (reset_reason != ESP_RST_POWERON && reset_reason != ESP_RST_EXT) {
        return;
    }
    if (!has_creds) {
        return;
    }
    ESP_LOGI(TAG, "cold boot; showing logo splash");
    splash_show_logo();
}

void app_main(void) {
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "boot; reset_reason=%d wakeup_cause=%d",
             reset_reason, esp_sleep_get_wakeup_cause());

    /* PMIC first: brings up the panel/SD/codec rails and gives us
     * battery telemetry. If this fails the device is still functional
     * on USB power -- heartbeat will report 0 mV (= unknown). */
    if (pmic_init() != ESP_OK) {
        ESP_LOGW(TAG, "PMIC init failed; continuing without battery telemetry");
    }

    /* Start the BOOT-button watcher as early as possible so even a press
     * during the multi-second WiFi connect is captured. */
    s_btn_events = xEventGroupCreate();
    xTaskCreate(button_watch_task, "btn_watch", 3072, NULL,
                /* priority */ 4, NULL);

    ESP_ERROR_CHECK(wifi_manager_init());

    /* Cold-boot splash: only when we have creds (otherwise the portal
     * splash will be shown next). A long-press during cold boot still
     * wins later; a tap before the splash starts is recorded and
     * forces a refresh after WiFi connects. */
    bool have_creds = wifi_creds_present();
    maybe_show_splash(reset_reason, have_creds);

    if (!have_creds) {
        run_provisioning_then_reboot();
        return;
    }

    esp_err_t err = wifi_sta_connect_stored();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "STA connect failed (%s); falling back to portal",
                 esp_err_to_name(err));
        run_provisioning_then_reboot();
        return;
    }

    /* Long-press detected at any point so far? Switch into the LAN
     * settings editor instead of the paint cycle. Long-press wins over
     * any pending tap. */
    if (button_hold_seen()) {
        ESP_LOGI(TAG, "settings mode (BOOT long-press): serving LAN editor + mDNS");
        if (settings_server_run_blocking() == ESP_OK) {
            ESP_LOGI(TAG, "settings saved; rebooting to apply");
            esp_restart();
        }
        ESP_LOGI(TAG, "settings editor timed out; back to sleep");
        wifi_sta_stop();
        sleep_forever_or_until_timer();
        return;
    }

    int sleep_s = load_sleep_interval_s();

    /* NTP carries across deep sleep via the RTC, so deep-sleep timer
     * wakes return instantly with cached time. Cold boots (power-on /
     * external reset) force a fresh exchange -- that's the recovery
     * path for an NTP-poisoned RTC (e.g. a router intercepting UDP/123
     * and returning a stale timestamp). 5 s window for the first sync
     * is empirically enough on residential WiFi. If the exchange
     * fails, sleep_until is omitted from the heartbeat and the
     * server's smart-sync scheduler falls back to its tolerance
     * window. */
    bool cold_boot = (reset_reason == ESP_RST_POWERON ||
                      reset_reason == ESP_RST_EXT);
    ensure_time_synced(5000, cold_boot);

    /* Fetch the retained URL only -- the heartbeat publishes at the END
     * (after paint, if any) so its sleep_until reflects the actual sleep
     * about to start instead of leading it by ~30 s of paint time. */
    mqtt_job_t job;
    err = mqtt_fetch_retained(&job);
    bool have_url = (err == ESP_OK && job.url[0]);
    if (!have_url) {
        ESP_LOGI(TAG, "no retained job (%s)", esp_err_to_name(err));
    }

    /* A long-press received during the MQTT step still flips us into
     * settings mode -- the user might press BOOT while the wake is in
     * flight specifically to grab the device's attention. */
    if (button_hold_seen()) {
        ESP_LOGI(TAG, "BOOT long-press during MQTT; switching to settings mode");
        if (settings_server_run_blocking() == ESP_OK) {
            esp_restart();
        }
        wifi_sta_stop();
        sleep_forever_or_until_timer();
        return;
    }

    bool need_paint = false;
    char hash[65];
    if (have_url) {
        sha256_hex(job.url, hash);
        if (button_tap_seen()) {
            ESP_LOGI(TAG, "BOOT tap detected; forcing refresh (skipping hash check)");
            need_paint = true;
        } else if (hash_matches_stored(hash)) {
            ESP_LOGI(TAG, "url unchanged since last render; skipping refresh");
        } else {
            need_paint = true;
        }
    }

    if (need_paint) {
        fetched_image_t img;
        err = image_fetch(job.url, &img);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "fetch failed: %s", esp_err_to_name(err));
            publish_heartbeat(sleep_s, reset_reason);   /* wifi still up */
            wifi_sta_stop();
            sleep_forever_or_until_timer();
            return;
        }

        uint8_t *frame = NULL;
        err = image_decode_to_frame(&img, job.url, &frame);
        free(img.data);
        if (err != ESP_OK || !frame) {
            ESP_LOGE(TAG, "decode failed: %s", esp_err_to_name(err));
            publish_heartbeat(sleep_s, reset_reason);   /* wifi still up */
            wifi_sta_stop();
            sleep_forever_or_until_timer();
            return;
        }

        /* Free WiFi for the ~25 s panel refresh -- the single biggest
         * battery saving in the render path (~80 mA otherwise). */
        wifi_sta_stop();

        ESP_ERROR_CHECK(epd_port_init());
        epd_display(frame);
        epd_sleep();
        free(frame);

        store_hash(hash);

        /* Bring WiFi back up just long enough to publish the final
         * heartbeat. ~3-5 s x ~80 mA = ~0.07-0.11 mAh extra per render
         * wake -- the cost of wall-clock-accurate sleep_until for
         * Tesserae's smart-sync scheduler. */
        ESP_LOGI(TAG, "render OK; reconnecting wifi for post-paint heartbeat");
        if (wifi_sta_connect_stored() == ESP_OK) {
            publish_heartbeat(sleep_s, reset_reason);
        } else {
            ESP_LOGW(TAG, "post-paint wifi reconnect failed; heartbeat skipped");
        }
        wifi_sta_stop();
        sleep_forever_or_until_timer();
        return;
    }

    /* Hash-skip or no-URL path: WiFi+MQTT just came up for the fetch
     * and the radio is still on; publish heartbeat without a reconnect
     * cycle. */
    publish_heartbeat(sleep_s, reset_reason);
    wifi_sta_stop();
    sleep_forever_or_until_timer();
}
