/* Waveshare 7.3" Spectra E6 (ED2208-GCA) e-paper driver.
 *
 * Protocol-faithfully ported from aitjcize/esp32-photoframe
 *   (components/epaper_driver_ed2208_gca/src/driver_ed2208_gca.c).
 * The shape of the SPI choreography matters for this panel:
 *
 *   1. Each command goes out via the SPI peripheral's command phase
 *      (SPI_TRANS_VARIABLE_CMD) so cmd + short data fit in one CS
 *      window without manually prepending the cmd byte to the data
 *      stream.
 *   2. The frame buffer is streamed in 128-byte chunks, each chunk
 *      copied from PSRAM into a stack-local internal-RAM buffer
 *      before the SPI write. PSRAM is unreliable as a direct SPI DMA
 *      source on ESP32-S3, so the round-trip through internal RAM is
 *      mandatory, not optional.
 *   3. Refresh order is RESET -> init -> DTM -> data -> PON -> DRF ->
 *      POF -> DSLP. Issuing PON before the frame data is sent leaves
 *      the boost rails pre-charged with nothing to display, and they
 *      can droop before DRF runs.
 *   4. DSLP (0x07, 0xA5) parks the panel after every paint; the next
 *      paint re-runs the whole init sequence.
 *   5. wait_busy starts with a 10 ms settle delay so we never sample
 *      BUSY before the panel has had a chance to assert it.
 *
 * The init opcodes and parameter bytes are panel-specific and lifted
 * from the Waveshare ED2208-GCA datasheet (also matching the
 * xiaozhi-esp32 reference and aitjcize's driver byte for byte).
 */

#include "epd_driver.h"
#include "pmic.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "epd";

/* --- panel command opcodes (Spectra 6 / ED2208-GCA) --- */
#define EPD_CMD_PSR          0x00   /* panel setting                  */
#define EPD_CMD_PWR          0x01   /* power setting                  */
#define EPD_CMD_POF          0x02   /* power off                      */
#define EPD_CMD_PFS          0x03   /* power frame settings           */
#define EPD_CMD_PON          0x04   /* power on                       */
#define EPD_CMD_BTST1        0x05   /* booster soft start 1           */
#define EPD_CMD_BTST2        0x06   /* booster soft start 2           */
#define EPD_CMD_DSLP         0x07   /* deep sleep                     */
#define EPD_CMD_BTST3        0x08   /* booster soft start 3           */
#define EPD_CMD_DTM          0x10   /* data transfer (frame data)     */
#define EPD_CMD_DRF          0x12   /* display refresh                */
#define EPD_CMD_PLL          0x30   /* PLL control                    */
#define EPD_CMD_CDI          0x50   /* VCOM/data interval setting     */
#define EPD_CMD_TCON         0x60   /* TCON setting                   */
#define EPD_CMD_TRES         0x61   /* resolution (800 x 480)         */
#define EPD_CMD_T_VDCS       0x84   /* T_VDCS                          */
#define EPD_CMD_CMDH         0xAA   /* required init preamble         */
#define EPD_CMD_PWS          0xE3   /* power saving                   */

#define EPD_DSLP_PARAM       0xA5   /* magic deep-sleep arg            */

/* Frame buffer source lives in PSRAM (image_decode allocates with
 * MALLOC_CAP_SPIRAM). PSRAM as a direct SPI DMA source is flaky on
 * ESP32-S3, so we copy each chunk through a small internal-RAM
 * stack buffer before the SPI write. 128 bytes matches aitjcize and
 * is small enough to keep stack pressure trivial while still
 * delivering ~50 KB/s after CS overhead. */
#define DATA_CHUNK_SIZE      128

/* For init sequences we send short data (<=16 bytes) inside the same
 * CS window as the command, with the cmd byte going via the SPI
 * peripheral's command phase. */
#define INIT_DATA_MAX        16

static spi_device_handle_t s_spi;
static bool s_port_inited = false;

/* ---------- SPI helpers ---------- */

/* Send one variable-length data burst inside an already-open CS
 * window. Caller manages CS and DC. */
static esp_err_t spi_write_data(const uint8_t *data, size_t len) {
    while (len > 0) {
        size_t chunk = (len > 4092) ? 4092 : len;
        spi_transaction_t t = {0};
        t.length    = chunk * 8;
        t.tx_buffer = data;
        esp_err_t err = spi_device_polling_start(s_spi, &t, portMAX_DELAY);
        if (err == ESP_OK) {
            err = spi_device_polling_end(s_spi, portMAX_DELAY);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi data tx failed: %s", esp_err_to_name(err));
            return err;
        }
        data += chunk;
        len  -= chunk;
    }
    return ESP_OK;
}

/* Send (cmd) and optionally (data) in a single CS window. The cmd byte
 * goes via the SPI peripheral's command phase (variable command size
 * = 8 bits, per-transaction). The data bytes are copied to a local
 * stack buffer first so we never hand the SPI driver a PSRAM source. */
static void cmd_data(uint8_t cmd, const uint8_t *data, size_t len) {
    uint8_t local[INIT_DATA_MAX];
    if (len > sizeof(local)) {
        ESP_LOGE(TAG, "cmd_data: %u bytes exceeds %u-byte limit",
                 (unsigned) len, (unsigned) sizeof(local));
        return;
    }
    if (data && len) {
        memcpy(local, data, len);
    }

    gpio_set_level(EPD_PIN_DC, 0);                /* command phase   */
    spi_device_acquire_bus(s_spi, portMAX_DELAY);
    gpio_set_level(EPD_PIN_CS, 0);

    spi_transaction_ext_t cmd_t = {
        .command_bits = 8,
        .base = {
            .flags = SPI_TRANS_VARIABLE_CMD,
            .cmd   = cmd,
        },
    };
    esp_err_t err = spi_device_polling_start(s_spi, &cmd_t.base, portMAX_DELAY);
    if (err == ESP_OK) {
        spi_device_polling_end(s_spi, portMAX_DELAY);
    }

    if (err == ESP_OK && len > 0) {
        gpio_set_level(EPD_PIN_DC, 1);            /* data phase       */
        err = spi_write_data(local, len);
    }

    gpio_set_level(EPD_PIN_CS, 1);
    spi_device_release_bus(s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cmd 0x%02X failed: %s", cmd, esp_err_to_name(err));
    }
}

static void send_cmd(uint8_t cmd) {
    cmd_data(cmd, NULL, 0);
}

/* Stream the frame buffer in DATA_CHUNK_SIZE-byte chunks, each in its
 * own CS window, with the source bytes routed through a stack-local
 * buffer so the SPI DMA never reads directly from PSRAM.
 *
 * Rotates the frame 180 degrees as it streams: the panel is mounted in
 * the PhotoPainter case 180 degrees from natural reading orientation
 * (front of case = bottom of the panel chip's address space), so a
 * frame composed top-left-first in source bytes would paint upside
 * down without this. The rotation is two transformations applied to
 * every byte:
 *
 *   - panel byte i is pulled from source byte (total - 1 - i)
 *     (reverses both row order and column-pair order in one shot,
 *     because the buffer is a flat scanline-major array).
 *   - each byte's high and low nibbles are swapped, which flips the
 *     two pixels packed inside that byte horizontally.
 *
 * Combined, that's a true 180 degree rotation. Cost is one byte read
 * + one byte write per output byte during the chunk copy -- effectively
 * free relative to the SPI clocking. */
static void send_buffer(const uint8_t *frame, size_t len) {
    uint8_t local[DATA_CHUNK_SIZE];

    ESP_LOGI(TAG, "Sending %u bytes in %u-byte chunks (rotated 180)",
             (unsigned) len, (unsigned) DATA_CHUNK_SIZE);

    /* Walk the source buffer from its end backwards, one chunk at a time. */
    const uint8_t *src_end = frame + len;

    while (len > 0) {
        size_t chunk = (len > DATA_CHUNK_SIZE) ? DATA_CHUNK_SIZE : len;
        const uint8_t *src = src_end - chunk;
        for (size_t i = 0; i < chunk; i++) {
            uint8_t b = src[chunk - 1 - i];
            local[i] = (uint8_t)((b << 4) | (b >> 4));
        }

        gpio_set_level(EPD_PIN_DC, 1);
        spi_device_acquire_bus(s_spi, portMAX_DELAY);
        gpio_set_level(EPD_PIN_CS, 0);
        spi_write_data(local, chunk);
        gpio_set_level(EPD_PIN_CS, 1);
        spi_device_release_bus(s_spi);

        src_end -= chunk;
        len     -= chunk;
    }
    ESP_LOGI(TAG, "Buffer send complete");
}

/* Wait for BUSY (active-low) to go high. Start with a small settle
 * delay so we don't sample before the panel has even asserted BUSY.
 * Returns false on timeout. */
#define EPD_WAIT_IDLE_TIMEOUT_MS  60000

static bool wait_idle(const char *label) {
    vTaskDelay(pdMS_TO_TICKS(10));
    int waited_ms = 10;
    while (gpio_get_level(EPD_PIN_BUSY) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited_ms += 10;
        if (waited_ms >= EPD_WAIT_IDLE_TIMEOUT_MS) {
            ESP_LOGW(TAG, "[%s] BUSY timeout after %d s",
                     label, EPD_WAIT_IDLE_TIMEOUT_MS / 1000);
            return false;
        }
    }
    return true;
}

static void hw_reset(void) {
    gpio_set_level(EPD_PIN_RST, 1); vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(EPD_PIN_RST, 0); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(EPD_PIN_RST, 1); vTaskDelay(pdMS_TO_TICKS(50));
}

/* ---------- public API ---------- */

esp_err_t epd_port_init(void) {
    if (s_port_inited) {
        return ESP_OK;
    }

    gpio_config_t out = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << EPD_PIN_RST) |
                        (1ULL << EPD_PIN_DC)  |
                        (1ULL << EPD_PIN_CS),
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    gpio_config_t in = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << EPD_PIN_BUSY),
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    gpio_set_level(EPD_PIN_CS,  1);
    gpio_set_level(EPD_PIN_DC,  0);
    gpio_set_level(EPD_PIN_RST, 1);

    spi_bus_config_t bus = {
        .miso_io_num     = -1,
        .mosi_io_num     = EPD_PIN_MOSI,
        .sclk_io_num     = EPD_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    spi_device_interface_config_t dev = {
        .clock_speed_hz = EPD_SPI_HZ,
        .mode           = 0,
        .spics_io_num   = -1,                            /* manual CS    */
        .queue_size     = 1,
        .flags          = SPI_DEVICE_HALFDUPLEX |
                          SPI_DEVICE_NO_DUMMY,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EPD_SPI_HOST, &bus, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(EPD_SPI_HOST, &dev, &s_spi));

    s_port_inited = true;
    return ESP_OK;
}

/* Send the init register sequence. Callable as part of either a full
 * paint (epd_display / epd_clear / splash) or stand-alone (which we
 * don't bother to expose -- the paint functions always re-init from
 * scratch because the panel was put to deep sleep at the end of the
 * previous one). */
static void send_init_sequence(void) {
    pmic_rails_set(true);   /* no-op if PMIC I2C is dead; harmless     */

    hw_reset();
    (void) wait_idle("reset");

    cmd_data(EPD_CMD_CMDH,   (uint8_t[]){0x49, 0x55, 0x20, 0x08, 0x09, 0x18}, 6);
    cmd_data(EPD_CMD_PWR,    (uint8_t[]){0x3F},                                1);
    cmd_data(EPD_CMD_PSR,    (uint8_t[]){0x5F, 0x69},                          2);
    cmd_data(EPD_CMD_PFS,    (uint8_t[]){0x00, 0x54, 0x00, 0x44},              4);
    cmd_data(EPD_CMD_BTST1,  (uint8_t[]){0x40, 0x1F, 0x1F, 0x2C},              4);
    cmd_data(EPD_CMD_BTST2,  (uint8_t[]){0x6F, 0x1F, 0x17, 0x49},              4);
    cmd_data(EPD_CMD_BTST3,  (uint8_t[]){0x6F, 0x1F, 0x1F, 0x22},              4);
    cmd_data(EPD_CMD_PLL,    (uint8_t[]){0x03},                                1);
    cmd_data(EPD_CMD_CDI,    (uint8_t[]){0x3F},                                1);
    cmd_data(EPD_CMD_TCON,   (uint8_t[]){0x02, 0x00},                          2);
    cmd_data(EPD_CMD_TRES,   (uint8_t[]){0x03, 0x20, 0x01, 0xE0},              4);
    cmd_data(EPD_CMD_T_VDCS, (uint8_t[]){0x01},                                1);
    cmd_data(EPD_CMD_PWS,    (uint8_t[]){0x2F},                                1);
}

/* Complete refresh cycle: reset -> init -> DTM -> data -> PON -> DRF
 * -> POF -> DSLP. Each step waits for BUSY to clear before the next
 * (with EPD_WAIT_IDLE_TIMEOUT_MS bound to keep the wake responsive
 * even if the panel hangs). */
static void display_cycle(const uint8_t *frame, size_t len) {
    send_init_sequence();
    (void) wait_idle("init");

    send_cmd(EPD_CMD_DTM);
    send_buffer(frame, len);
    (void) wait_idle("data");

    send_cmd(EPD_CMD_PON);
    (void) wait_idle("power_on");

    cmd_data(EPD_CMD_DRF, (uint8_t[]){0x00}, 1);
    (void) wait_idle("refresh");

    cmd_data(EPD_CMD_POF, (uint8_t[]){0x00}, 1);
    (void) wait_idle("power_off");

    /* Park the panel in deep sleep until the next epd_init() call. */
    cmd_data(EPD_CMD_DSLP, (uint8_t[]){EPD_DSLP_PARAM}, 1);
    ESP_LOGI(TAG, "refresh done");
}

void epd_clear(uint8_t color) {
    uint8_t *buf = heap_caps_malloc(EPD_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "OOM allocating %u-byte clear buffer",
                 (unsigned) EPD_BUF_BYTES);
        return;
    }
    memset(buf, (color << 4) | color, EPD_BUF_BYTES);
    display_cycle(buf, EPD_BUF_BYTES);
    free(buf);
}

void epd_display(const uint8_t *image) {
    display_cycle(image, EPD_BUF_BYTES);
}

void epd_show_color_bars(void) {
    /* Splash: 6 horizontal bands in palette order. Build the whole
     * 192000-byte buffer in PSRAM then push it through the standard
     * display cycle -- the per-chunk stack-copy in send_buffer handles
     * the PSRAM-source case. */
    static const uint8_t palette[6] = {
        EPD_COL_BLACK, EPD_COL_WHITE,  EPD_COL_YELLOW,
        EPD_COL_RED,   EPD_COL_BLUE,   EPD_COL_GREEN,
    };
    uint8_t *buf = heap_caps_malloc(EPD_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "OOM allocating splash buffer");
        return;
    }
    const size_t ROW_BYTES = EPD_WIDTH / 2;
    size_t base_band_h = EPD_HEIGHT / 6;
    size_t last_band_h = base_band_h + (EPD_HEIGHT % 6);
    size_t row = 0;
    for (int b = 0; b < 6; b++) {
        size_t band_h = (b == 5) ? last_band_h : base_band_h;
        uint8_t packed = (palette[b] << 4) | palette[b];
        for (size_t r = 0; r < band_h; r++) {
            memset(buf + row * ROW_BYTES, packed, ROW_BYTES);
            row++;
        }
    }
    display_cycle(buf, EPD_BUF_BYTES);
    free(buf);
}

void epd_show_palette_sweep(void) {
    /* Diagnostic: 8 horizontal bands of HEIGHT/8 = 60 rows each. */
    uint8_t *buf = heap_caps_malloc(EPD_BUF_BYTES, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "OOM allocating palette-sweep buffer");
        return;
    }
    const size_t ROW_BYTES = EPD_WIDTH / 2;
    const size_t BAND_H    = EPD_HEIGHT / 8;
    size_t row = 0;
    for (uint8_t n = 0; n < 8; n++) {
        uint8_t packed = (n << 4) | n;
        for (size_t r = 0; r < BAND_H; r++) {
            memset(buf + row * ROW_BYTES, packed, ROW_BYTES);
            row++;
        }
    }
    display_cycle(buf, EPD_BUF_BYTES);
    free(buf);
}

void epd_sleep(void) {
    /* display_cycle already left the panel in deep sleep + powered off
     * (DSLP at the end of every paint), so all we need to do here is
     * deassert RST to keep the line low across deep sleep.
     *
     * TODO(battery): also call pmic_rails_set(false) to drop the
     * AXP2101 ALDOs and shed the ~10 mA the panel/SD/codec rails draw
     * even with the panel parked. Currently disabled out of caution --
     * we want to confirm on real hardware that the wake-up path
     * (IRQ pulse + bus rescue in pmic_init) reliably re-enables the
     * rails on the next boot. Cheap to enable once we've validated:
     * one line here plus matching pmic_rails_set(true) after pmic_init
     * (which already happens inside display_cycle / send_init_sequence). */
    gpio_set_level(EPD_PIN_RST, 0);
}
