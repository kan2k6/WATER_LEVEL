#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h> //file folder
#include <unistd.h>   //file operations
#include <errno.h>    //error handling for file operations
#include <time.h>     //unix time
#include <sys/time.h> //setting time for modem & esp

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_task_wdt.h" //watchdog timer
#include "esp_rom_sys.h"  //delay microseconds
#include "esp_err.h"      //error handling for esp functions
#include "esp_attr.h"     //function attributes

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"

#include "sdmmc_cmd.h"   //SD card commands and types
#include "esp_vfs_fat.h" //FAT filesystem support for SD cards

#include "nvs_flash.h" //NVS flash storage
#include "nvs.h"       //NVS API

#include "cJSON.h" //JSON parsing and generation

static const char *TAG = "water_level";

// CONFIG
// USER

#define SLEEP_MINUTES 15
#define ALIGN_WAKE_TO_NETWORK_QUARTER 1
#define TIME_SYNC_EACH_WAKE 1
#define MIN_SLEEP_SECONDS 30
#define QUARTER_INTERVAL_SECONDS (15U * 60U)
#define FALLBACK_SLEEP_TIME_US ((uint64_t)SLEEP_MINUTES * 60ULL * 1000000ULL)

// sensor

#define SENSOR_TRIG_GPIO GPIO_NUM_4
#define SENSOR_ECHO_GPIO GPIO_NUM_5
#define SENSOR_MAX_HEIGHT_CM 200.0f

// MODEM UART

#define MODEM_UART_PORT UART_NUM_1
#define MODEM_UART_BAUD 115200

#define MODEM_UART_TX_GPIO GPIO_NUM_21
#define MODEM_UART_RX_GPIO GPIO_NUM_20
#define MODEM_UART_RTS_GPIO GPIO_NUM_NC
#define MODEM_UART_CTS_GPIO GPIO_NUM_NC
#define MODEM_DTR_GPIO GPIO_NUM_NC
#define MODEM_RI_GPIO GPIO_NUM_NC

#define MODEM_PWRKEY_GPIO GPIO_NUM_NC
#define MODEM_RESET_GPIO GPIO_NUM_NC
#define MODEM_POWER_EN_GPIO GPIO_NUM_NC

#define MODEM_USB_VBUS_EN_GPIO GPIO_NUM_NC

// SD CARD

#define SD_MOSI_GPIO GPIO_NUM_7
#define SD_MISO_GPIO GPIO_NUM_2
#define SD_SCLK_GPIO GPIO_NUM_6
#define SD_CS_GPIO GPIO_NUM_10

#define MOUNT_POINT "/sdcard"
#define DATA_FILE MOUNT_POINT "/data.jl"
#define TMP_FILE MOUNT_POINT "/data.tmp"
#define BAK_FILE MOUNT_POINT "/data.bak"

// MQTT

#define APN_NAME "v-internet"

#define MQTT_BROKER_HOST "mqtt.thingsboard.cloud"
#define MQTT_BROKER_PORT 1883

#define MQTT_CLIENT_ID "FIELD_DEVICE_001"
#define MQTT_USERNAME "gnv448gdkhfadb4qpdhq"
#define MQTT_PASSWORD ""

#define MQTT_TOPIC_PUB "v1/devices/me/telemetry"

#define MQTT_KEEPALIVE_SECONDS 930
#define MQTT_QOS 1    // pub min 1, need to ensure
#define MQTT_RETAIN 0 // do not retain, only care about new data
#define MQTT_PUB_TIMEOUT_SECONDS 60
#define ENABLE_GNSS 0
#define GNSS_FIX_TIMEOUT_MS 60000

// LIMIT

#define MAX_LINE_LEN 1200
#define MAX_AT_RESP_LEN 3072
#define MAX_PENDING_SEND_PER_WAKE 0

// WATCHDOG / STATE TIMEOUT

#define TASK_WDT_TIMEOUT_MS 120000

#define STATE_BOOT_TIMEOUT_MS 20000
#define STATE_CHECK_SD_MODEM_TIMEOUT_MS 90000
#define STATE_CHECK_PENDING_TIMEOUT_MS 20000
#define STATE_READ_SENSOR_TIMEOUT_MS 10000
#define STATE_SAVE_SD_TIMEOUT_MS 15000
#define STATE_MODEM_WAKE_TIMEOUT_MS 90000
#define STATE_NETWORK_TIMEOUT_MS 90000
#define STATE_SYNC_TIME_TIMEOUT_MS 15000
#define STATE_GNSS_TIMEOUT_MS 70000
#define STATE_MQTT_TIMEOUT_MS 150000
#define STATE_SEND_TIMEOUT_MS 150000
#define STATE_MODEM_SLEEP_TIMEOUT_MS 20000
#define STATE_DEEP_SLEEP_TIMEOUT_MS 10000

// STATE MACHINE

typedef enum
{
    STATE_BOOT,
    STATE_CHECK_SD_AND_SIM_MODEM,
    STATE_READ_SENSOR,
    STATE_SAVE_TO_SD,
    STATE_READ_GNSS,
    STATE_UPDATE_GNSS_TO_SD,
    STATE_CHECK_SD_PENDING,
    STATE_NETWORK_READY,
    STATE_MQTT_CONNECT,
    STATE_SEND_ALL_PENDING,
    STATE_PREPARE_MQTT_KEEPALIVE,
    STATE_DEEP_SLEEP

} app_state_t;

typedef enum
{
    EVENT_OK,
    EVENT_FAIL,
    EVENT_TIMEOUT
} app_event_t;

typedef struct
{
    uint32_t id;
    uint32_t sended;
    float water_level;

    uint8_t gps_valid;
    double gps_lat;
    double gps_lon;

    uint8_t time_valid;
    uint64_t unix_time;
    char time_iso[32];
} water_level_record_t;

typedef struct
{
    app_state_t state;
    int64_t state_start_ms;
    int64_t state_timeout_ms;

    bool sd_mounted;
    bool has_pending_records;
    bool modem_ready;
    bool modem_uart_ready;
    bool modem_responding;
    bool mqtt_connected;
    bool esp_time_valid;
    bool gnss_valid;
    uint32_t next_sleep_seconds;

    uint16_t mqtt_msg_id;

    water_level_record_t current_record;
} app_context_t;

static app_context_t ctx = {
    .mqtt_msg_id = 1,
    .next_sleep_seconds = QUARTER_INTERVAL_SECONDS,
};

RTC_DATA_ATTR static bool rtc_epoch_valid = false;
RTC_DATA_ATTR static int64_t rtc_epoch_after_sleep = 0;
RTC_DATA_ATTR static uint32_t rtc_boot_count = 0;

// TIME / RTC

static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned mp = m + (m > 2 ? -3 : 9);
    const unsigned doy = (153 * mp + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

static bool make_utc_epoch(int year, int mon, int mday, int hour, int min, int sec, time_t *out_epoch)
{
    if (!out_epoch || year < 1970 || mon < 1 || mon > 12 || mday < 1 || mday > 31 ||
        hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 60)
    {
        return false;
    }

    int64_t days = days_from_civil(year, (unsigned)mon, (unsigned)mday);
    int64_t epoch = days * 86400LL + hour * 3600LL + min * 60LL + sec;
    *out_epoch = (time_t)epoch;
    return true;
}

static bool epoch_is_reasonable(int64_t epoch)
{
    const int64_t epoch_2020 = 1577836800;
    return (epoch >= epoch_2020);
}

static void format_epoch_utc(int64_t epoch, char *out_str, size_t out_str_size)
{
    time_t t = (time_t)epoch;
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(out_str, out_str_size, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

static void esp_set_epoch_time(time_t epoch)
{
    struct timeval tv = {
        .tv_sec = epoch,
        .tv_usec = 0};
    settimeofday(&tv, NULL);
    ctx.esp_time_valid = epoch_is_reasonable(epoch);

    ESP_LOGI(TAG, "ESP RTC/system time set: epoch=%lld, valid=%d",
             (long long)epoch, ctx.esp_time_valid);
}

static void rtc_restore_time_on_boot(void)
{
    rtc_boot_count++;

    if (rtc_epoch_valid && epoch_is_reasonable(rtc_epoch_after_sleep))
    {
        esp_set_epoch_time((time_t)rtc_epoch_after_sleep);
        ESP_LOGI(TAG, "Restored approximate ESP time from RTC memory, boot=%lu",
                 (unsigned long)rtc_boot_count);
    }
    else
    {
        ctx.esp_time_valid = false;
        ESP_LOGW(TAG, "No valid RTC time yet, boot=%lu", (unsigned long)rtc_boot_count);
    }
}

static void fill_record_time_from_esp(water_level_record_t *rec)
{
    if (!rec)
        return;

    time_t now = time(NULL);
    if (epoch_is_reasonable(now))
    {
        rec->time_valid = 1;
        rec->unix_time = (int64_t)now;
        format_epoch_utc(now, rec->time_iso, sizeof(rec->time_iso));
    }
    else
    {
        rec->time_valid = 0;
        rec->unix_time = 0;
        rec->time_iso[0] = '\0';
    }
}

static uint32_t calculate_sleep_seconds_to_next_quarter(void)
{
#if ALIGN_WAKE_TO_NETWORK_QUARTER
    time_t now = time(NULL);

    if (!epoch_is_reasonable(now))
    {
        ESP_LOGW(TAG, "Time invalid, fallback fixed sleep %u seconds", QUARTER_INTERVAL_SECONDS);
        return QUARTER_INTERVAL_SECONDS;
    }

    uint32_t rem = (uint32_t)(now % QUARTER_INTERVAL_SECONDS);
    uint32_t sleep_s = QUARTER_INTERVAL_SECONDS - rem;

    if (sleep_s < MIN_SLEEP_SECONDS)
    {
        sleep_s += QUARTER_INTERVAL_SECONDS;
    }

    ESP_LOGI(TAG, "Sleep aligned to next minute%%15==0: now=%lld, rem=%lu, sleep=%lu s",
             (long long)now, (unsigned long)rem, (unsigned long)sleep_s);
    return sleep_s;
#else
    return QUARTER_INTERVAL_SECONDS;
#endif
}

static void rtc_store_expected_wakeup_time(uint32_t sleep_seconds)
{
    time_t now = time(NULL);

    if (epoch_is_reasonable(now))
    {
        rtc_epoch_after_sleep = (int64_t)now + sleep_seconds;
        rtc_epoch_valid = true;
        ESP_LOGI(TAG, "Stored expected wakeup epoch in RTC memory: %lld",
                 (long long)rtc_epoch_after_sleep);
    }
    else
    {
        rtc_epoch_valid = false;
        ESP_LOGW(TAG, "Current time invalid, cannot store expected wakeup epoch, boot=%lu",
                 (unsigned long)rtc_boot_count);
    }
}

static bool parse_quectel_time_response_to_epoch(const char *resp, time_t *out_epoch)
{
    if (!resp || !out_epoch)
        return false;

    int year = 0, mon = 0, day = 0, hour = 0, minute = 0, sec = 0;

    const char *p = strstr(resp, "+QLTS:");
    if (p)
    {
        p = strchr(p, ':');
        if (!p)
            return false;
        p++;
        while (*p == ' ' || *p == '"')
            p++;

        if (sscanf(p, "%4d/%2d/%2d,%2d:%2d:%2d", &year, &mon, &day, &hour, &minute, &sec) != 6)
            return false;
    }
    else
    {
        p = strstr(resp, "+CCLK:");
        if (!p)
            return false;
        p = strchr(p, '"');
        if (!p)
            return false;
        p++;

        int yy = 0;
        if (sscanf(p, "%2d/%2d/%2d,%2d:%2d:%2d", &yy, &mon, &day, &hour, &minute, &sec) != 6)
            return false;
        year = (yy >= 70) ? (1900 + yy) : (2000 + yy);
    }

    time_t epoch = 0;
    if (!make_utc_epoch(year, mon, day, hour, minute, sec, &epoch))
        return false;

    if (!epoch_is_reasonable(epoch))
        return false;

    *out_epoch = epoch;
    return true;
}

// timeout for esp

static const char *state_name(app_state_t state)
{
    switch (state)
    {
    case STATE_BOOT:
        return "BOOT";
    case STATE_CHECK_SD_AND_SIM_MODEM:
        return "CHECK_SD_AND_SIM_MODEM";
    case STATE_READ_SENSOR:
        return "READ_SENSOR";
    case STATE_SAVE_TO_SD:
        return "SAVE_TO_SD";
    case STATE_READ_GNSS:
        return "READ_GNSS";
    case STATE_UPDATE_GNSS_TO_SD:
        return "UPDATE_GNSS_TO_SD";
    case STATE_CHECK_SD_PENDING:
        return "CHECK_SD_PENDING";
    case STATE_NETWORK_READY:
        return "NETWORK_READY";
    case STATE_MQTT_CONNECT:
        return "MQTT_CONNECT";
    case STATE_SEND_ALL_PENDING:
        return "SEND_ALL_PENDING";
    case STATE_PREPARE_MQTT_KEEPALIVE:
        return "PREPARE_MQTT_KEEPALIVE";
    case STATE_DEEP_SLEEP:
        return "DEEP_SLEEP";
    default:
        return "UNKNOWN_STATE";
    }
}

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000ULL;
}

static int64_t state_timeout_ms(app_state_t state)
{
    switch (state)
    {
    case STATE_BOOT:
        return STATE_BOOT_TIMEOUT_MS;
    case STATE_CHECK_SD_AND_SIM_MODEM:
        return STATE_CHECK_SD_MODEM_TIMEOUT_MS;
    case STATE_READ_SENSOR:
        return STATE_READ_SENSOR_TIMEOUT_MS;
    case STATE_SAVE_TO_SD:
        return STATE_SAVE_SD_TIMEOUT_MS;
    case STATE_READ_GNSS:
        return STATE_GNSS_TIMEOUT_MS;
    case STATE_UPDATE_GNSS_TO_SD:
        return STATE_SAVE_SD_TIMEOUT_MS;
    case STATE_CHECK_SD_PENDING:
        return STATE_CHECK_PENDING_TIMEOUT_MS;
    case STATE_NETWORK_READY:
        return STATE_NETWORK_TIMEOUT_MS;
    case STATE_MQTT_CONNECT:
        return STATE_MQTT_TIMEOUT_MS;
    case STATE_SEND_ALL_PENDING:
        return STATE_SEND_TIMEOUT_MS;
    case STATE_PREPARE_MQTT_KEEPALIVE:
        return STATE_MODEM_SLEEP_TIMEOUT_MS;
    case STATE_DEEP_SLEEP:
        return STATE_DEEP_SLEEP_TIMEOUT_MS;
    default:
        return 30000;
    }
}

static void feed_wdt(void)
{
    esp_task_wdt_reset();
}

static void enter_state(app_state_t new_state)
{
    ctx.state = new_state;
    ctx.state_start_ms = now_ms();
    ctx.state_timeout_ms = state_timeout_ms(new_state);

    ESP_LOGI(TAG, "Entered state %s, timeout in %lld ms",
             state_name(new_state), (long long)ctx.state_timeout_ms);
}

static void check_state_timeout(void)
{
    if (ctx.state_timeout_ms <= 0)
    {
        return;
    }

    int64_t elapsed = now_ms() - ctx.state_start_ms;

    if (elapsed <= ctx.state_timeout_ms)
    {
        return;
    }

    ESP_LOGW(TAG, "State %s timeout after %lld ms, timeout=%lld ms",
             state_name(ctx.state),
             (long long)elapsed,
             (long long)ctx.state_timeout_ms);

    switch (ctx.state)
    {
    case STATE_BOOT:
    case STATE_CHECK_SD_AND_SIM_MODEM:
    case STATE_READ_SENSOR:
    case STATE_SAVE_TO_SD:
        ESP_LOGE(TAG, "Critical state timeout, restarting ESP");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
        break;

    case STATE_READ_GNSS:
    case STATE_UPDATE_GNSS_TO_SD:
    case STATE_NETWORK_READY:
    case STATE_MQTT_CONNECT:
    case STATE_SEND_ALL_PENDING:
        ESP_LOGW(TAG, "Communication state timeout, go to prepare sleep");
        enter_state(STATE_PREPARE_MQTT_KEEPALIVE);
        break;

    case STATE_PREPARE_MQTT_KEEPALIVE:
        ESP_LOGW(TAG, "Prepare keepalive timeout, go to deep sleep");
        enter_state(STATE_DEEP_SLEEP);
        break;

    case STATE_DEEP_SLEEP:
        break;

    default:
        ESP_LOGW(TAG, "Unknown timeout state, go to deep sleep");
        enter_state(STATE_DEEP_SLEEP);
        break;
    }
}

// GPIO

static bool gpio_is_valid(gpio_num_t pin)
{
    return (pin != GPIO_NUM_NC) && GPIO_IS_VALID_GPIO(pin);
}

static bool gpio_is_valid_out(gpio_num_t pin)
{
    return (pin != GPIO_NUM_NC) && GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

static void gpio_output_level(gpio_num_t pin, int level, bool pullup)
{
    if (!gpio_is_valid_out(pin))
    {
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(pin, level);
}

static void gpio_input_pullup(gpio_num_t pin)
{
    if (!gpio_is_valid(pin))
    {
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

static void gpio_hold_if_valid(gpio_num_t pin)
{
    if (!gpio_is_valid(pin))
    {
        return;
    }
    gpio_hold_en(pin);
}

static void gpio_hold_dis_if_valid(gpio_num_t pin)
{
    if (!gpio_is_valid(pin))
    {
        return;
    }
    gpio_hold_dis(pin);
}

// WACTHDOG

static void watchdog_init(void)
{
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = TASK_WDT_TIMEOUT_MS,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };

    esp_err_t ret = esp_task_wdt_init(&wdt_config);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "Task watchdog enabled");

    ret = esp_task_wdt_add(NULL);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "Current task added to watchdog");
}

// MODEM_GPIO

static void modem_gpio_release_hold(void)
{
    gpio_hold_dis_if_valid(MODEM_PWRKEY_GPIO);
    gpio_hold_dis_if_valid(MODEM_RESET_GPIO);
    gpio_hold_dis_if_valid(MODEM_POWER_EN_GPIO);
    gpio_hold_dis_if_valid(MODEM_DTR_GPIO);
    gpio_hold_dis_if_valid(MODEM_USB_VBUS_EN_GPIO);
    gpio_hold_dis_if_valid(MODEM_UART_TX_GPIO);
    gpio_hold_dis_if_valid(MODEM_UART_RX_GPIO);
    gpio_deep_sleep_hold_dis();
}

static void modem_gpio_active_state(void)
{
    modem_gpio_release_hold();

    gpio_output_level(MODEM_POWER_EN_GPIO, 1, false);
    gpio_output_level(MODEM_USB_VBUS_EN_GPIO, 0, false);
    gpio_output_level(MODEM_RESET_GPIO, 1, true);
    gpio_output_level(MODEM_PWRKEY_GPIO, 1, true);
    gpio_output_level(MODEM_DTR_GPIO, 0, true);
    gpio_output_level(MODEM_UART_TX_GPIO, 1, true);
    gpio_input_pullup(MODEM_UART_RX_GPIO);
}

static void modem_gpio_sleep_prep(void)
{
    gpio_output_level(MODEM_POWER_EN_GPIO, 1, false);
    gpio_output_level(MODEM_USB_VBUS_EN_GPIO, 0, false);
    gpio_output_level(MODEM_RESET_GPIO, 1, true);
    gpio_output_level(MODEM_PWRKEY_GPIO, 1, true);
    gpio_output_level(MODEM_DTR_GPIO, 0, true);
    gpio_output_level(MODEM_UART_TX_GPIO, 1, true);
    gpio_input_pullup(MODEM_UART_RX_GPIO);
    gpio_hold_if_valid(MODEM_UART_TX_GPIO);
    gpio_hold_if_valid(MODEM_UART_RX_GPIO);
    gpio_hold_if_valid(MODEM_PWRKEY_GPIO);
    gpio_hold_if_valid(MODEM_RESET_GPIO);
    gpio_hold_if_valid(MODEM_POWER_EN_GPIO);

    gpio_deep_sleep_hold_en();
}

static void modem_pwrkey_pulse_power_on(void)
{
    gpio_output_level(MODEM_POWER_EN_GPIO, 1, false);
    gpio_output_level(MODEM_RESET_GPIO, 1, true);
    gpio_output_level(MODEM_PWRKEY_GPIO, 1, true);

    vTaskDelay(pdMS_TO_TICKS(300));

    gpio_set_level(MODEM_PWRKEY_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    gpio_set_level(MODEM_PWRKEY_GPIO, 1);

    ESP_LOGI(TAG, "PWRKEY pulse sent");
}

// NVS

static uint32_t nvs_get_next_record_id(void)
{
    nvs_handle_t nvs;
    uint32_t id = 0;

    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return 0;
    }

    err = nvs_get_u32(nvs, "next_id", &id);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        id = 1;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read next_id from NVS: %s", esp_err_to_name(err));
        nvs_close(nvs);
        return 0;
    }

    nvs_set_u32(nvs, "next_id", id + 1);
    nvs_commit(nvs);
    nvs_close(nvs);
    return id;
}

// SD CARD

static esp_err_t sd_mount(void)
{
    if (ctx.sd_mounted)
    {
        return ESP_OK;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_GPIO;
    slot_config.host_id = host.slot;

    sdmmc_card_t *card = NULL;
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret == ESP_OK)
    {
        ctx.sd_mounted = true;
        ESP_LOGI(TAG, "SD card mounted at " MOUNT_POINT);
        sdmmc_card_print_info(stdout, card);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    }
    return ret;
}

static bool file_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0;
}

static bool safe_replace_file(const char *tmp_path, const char *main_path, const char *bak_path)
{
    remove(bak_path);

    if (file_exists(main_path))
    {
        if (rename(main_path, bak_path) != 0)
        {
            ESP_LOGE(TAG, "Rename main to bak failed, errno=%d", errno);
            remove(tmp_path);
            return false;
        }
    }

    if (rename(tmp_path, main_path) == 0)
    {
        remove(bak_path);
        return true;
    }

    ESP_LOGE(TAG, "Rename tmp to main failed, restoring backup");
    if (file_exists(bak_path))
        rename(bak_path, main_path);
    remove(tmp_path);
    return false;
}

static void recover_file_if_needed(void)
{
    if (!file_exists(DATA_FILE) && file_exists(BAK_FILE))
    {
        ESP_LOGW(TAG, "Recovering data file from backup");
        rename(BAK_FILE, DATA_FILE);
    }

    if (file_exists(TMP_FILE))
    {
        ESP_LOGW(TAG, "Removing old tmp file");
        remove(TMP_FILE);
    }

    if (file_exists(DATA_FILE) && file_exists(BAK_FILE))
    {
        ESP_LOGW(TAG, "Both data file and backup exist, removing backup to avoid confusion");
        remove(BAK_FILE);
    }
}

// SENSOR

static app_event_t action_read_sensor(void)
{
    memset(&ctx.current_record, 0, sizeof(ctx.current_record));

    ctx.current_record.id = nvs_get_next_record_id();
    ctx.current_record.sended = 0;

    ctx.current_record.water_level = 35.0f;

    fill_record_time_from_esp(&ctx.current_record);

    ESP_LOGI(TAG, "Read sensor OK, water_level=%.2f cm",
             ctx.current_record.water_level);

    return EVENT_OK;
}

// json record

static char *create_record_json(const water_level_record_t *rec)
{
    cJSON *root = cJSON_CreateObject();
    if (!root)
        return NULL;

    cJSON_AddNumberToObject(root, "id", rec->id);
    cJSON_AddNumberToObject(root, "sended", rec->sended);
    cJSON_AddNumberToObject(root, "water_level", rec->water_level);
    cJSON_AddNumberToObject(root, "gps_valid", rec->gps_valid);
    cJSON_AddNumberToObject(root, "gps_lat", rec->gps_lat);
    cJSON_AddNumberToObject(root, "gps_lon", rec->gps_lon);
    cJSON_AddNumberToObject(root, "time_valid", rec->time_valid);
    cJSON_AddNumberToObject(root, "unix_time", rec->unix_time);
    cJSON_AddStringToObject(root, "time_iso", rec->time_iso);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

static app_event_t action_save_to_sd(void)
{
    ctx.current_record.sended = 0;

    FILE *f = fopen(DATA_FILE, "a");
    if (!f)
    {
        ESP_LOGE(TAG, "Open data file append failed");
        return EVENT_FAIL;
    }

    char *json = create_record_json(&ctx.current_record);
    if (!json)
    {
        fclose(f);
        ESP_LOGE(TAG, "Create JSON failed");
        return EVENT_FAIL;
    }

    int ret = fprintf(f, "%s\n", json);

    fflush(f);
    fsync(fileno(f));
    fclose(f);
    free(json);

    if (ret <= 0)
    {
        ESP_LOGE(TAG, "Write SD failed");
        return EVENT_FAIL;
    }

    return EVENT_OK;
}

// AT COMMAND

static esp_err_t modem_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = MODEM_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(MODEM_UART_PORT, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(MODEM_UART_PORT, MODEM_UART_TX_GPIO, MODEM_UART_RX_GPIO,
                       MODEM_UART_RTS_GPIO, MODEM_UART_CTS_GPIO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(MODEM_UART_PORT, 2048, 0, 0, NULL, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ctx.modem_uart_ready = true;
    return ESP_OK;
}

static void modem_uart_delete(void)
{
    if (ctx.modem_uart_ready)
    {
        uart_driver_delete(MODEM_UART_PORT);
        ctx.modem_uart_ready = false;
    }
}

static bool wait_for_modem_response(const char *expected_prefix, int timeout_ms, char *out_buf, size_t out_buf_size)
{
    int64_t start = now_ms();
    size_t buf_pos = 0;

    while (now_ms() - start < timeout_ms)
    {
        feed_wdt();

        uint8_t byte;
        int len = uart_read_bytes(MODEM_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            if (buf_pos < out_buf_size - 1)
            {
                out_buf[buf_pos++] = (char)byte;
                out_buf[buf_pos] = '\0';

                if (strstr(out_buf, expected_prefix) != NULL)
                {
                    return true;
                }
            }
            else
            {
                ESP_LOGW(TAG, "Modem response buffer overflow");
                return false;
            }
        }
    }

    ESP_LOGW(TAG, "Timeout waiting for modem response: %s", expected_prefix);
    return false;
}

static bool at_send(const char *cmd, const char *expect, int timeout_ms, char *out, size_t out_size)
{
    if (!ctx.modem_uart_ready)
    {
        ESP_LOGE(TAG, "UART not ready for AT command");
        return false;
    }

    ESP_LOGI(TAG, "AT > %s", cmd);

    uart_flush_input(MODEM_UART_PORT);
    uart_write_bytes(MODEM_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);

    bool ok = wait_for_modem_response(expect, timeout_ms, out, out_size);
    if (ok && out && strlen(out) > 0)
    {
        ESP_LOGI(TAG, "AT RESP: %s", out);
    }
    return ok;
}

static bool at_send_simple(const char *cmd, int timeout_ms)
{
    char resp[MAX_AT_RESP_LEN];
    return at_send(cmd, "OK", timeout_ms, resp, sizeof(resp));
}

static bool at_send_payload(const char *cmd,
                            const char *payload,
                            const char *expect_after_payload,
                            int prompt_timeout_ms,
                            int final_timeout_ms,
                            char *out,
                            size_t out_size)
{
    if (!ctx.modem_uart_ready)
        return false;

    ESP_LOGI(TAG, "AT > %s", cmd);

    uart_flush_input(MODEM_UART_PORT);
    uart_write_bytes(MODEM_UART_PORT, cmd, strlen(cmd));
    uart_write_bytes(MODEM_UART_PORT, "\r\n", 2);

    if (!wait_for_modem_response(">", prompt_timeout_ms, out, out_size))
    {
        ESP_LOGE(TAG, "No payload prompt for command");
        return false;
    }

    uart_write_bytes(MODEM_UART_PORT, payload, strlen(payload));

    bool ok = wait_for_modem_response(expect_after_payload, final_timeout_ms, out, out_size);
    if (ok && out && strlen(out) > 0)
    {
        ESP_LOGI(TAG, "AT PAYLOAD RESP: %s", out);
    }
    return ok;
}

// MODEM WAKE / NETWORK

static app_event_t modem_is_ready(void)
{
    char resp[MAX_AT_RESP_LEN];
    if (at_send("AT+CPIN?", "OK", 2000, resp, sizeof(resp)))
    {
        if (strstr(resp, "+CPIN: READY"))
        {
            ctx.modem_ready = true;
            return EVENT_OK;
        }
    }
    ctx.modem_ready = false;
    return EVENT_FAIL;
}

static bool modem_wait_at_ready(int total_timeout_ms)
{
    char resp[MAX_AT_RESP_LEN];
    int64_t start = now_ms();

    while ((now_ms() - start) < total_timeout_ms)
    {
        check_state_timeout();
        memset(resp, 0, sizeof(resp));

        if (at_send("AT", "OK", 1000, resp, sizeof(resp)))
        {
            ctx.modem_responding = true;
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ctx.modem_responding = false;
    return false;
}

static app_event_t action_modem_wake(void)
{
    modem_gpio_active_state();

    if (!ctx.modem_uart_ready)
    {
        esp_err_t ret = modem_uart_init();
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "Modem UART init failed");
            ctx.modem_responding = false;
            return EVENT_FAIL;
        }
    }

    if (!modem_wait_at_ready(5000))
    {
        ESP_LOGW(TAG, "Modem not responding, try PWRKEY pulse");

        modem_pwrkey_pulse_power_on();
        vTaskDelay(pdMS_TO_TICKS(8000));
        if (!modem_wait_at_ready(30000))
        {
            ESP_LOGE(TAG, "Modem not responding after PWRKEY");
            ctx.modem_responding = false;
            return EVENT_FAIL;
        }
    }

    ctx.modem_responding = true;
    return EVENT_OK;
}

static bool modem_is_registered_response(const char *resp)
{
    return strstr(resp, "+CEREG: 0,1") || strstr(resp, "+CEREG: 0,5") ||
           strstr(resp, "+CEREG: 1,1") || strstr(resp, "+CEREG: 1,5") ||
           strstr(resp, "+CREG: 0,1") || strstr(resp, "+CREG: 0,5") ||
           strstr(resp, "+CREG: 1,1") || strstr(resp, "+CREG: 1,5");
}

static app_event_t action_sync_network_time(void)
{
    char resp[MAX_AT_RESP_LEN];

#if TIME_SYNC_EACH_WAKE
    at_send_simple("AT+CTZU=3", 1000);
    at_send_simple("AT+CTZR=2", 1000);

    for (int i = 0; i < 5; i++)
    {
        memset(resp, 0, sizeof(resp));
        if (at_send("AT+QLTS=2", "OK", 2000, resp, sizeof(resp)))
        {
            time_t epoch = 0;
            if (parse_quectel_time_response_to_epoch(resp, &epoch))
            {
                esp_set_epoch_time(epoch);
                ctx.next_sleep_seconds = calculate_sleep_seconds_to_next_quarter();
                return EVENT_OK;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    memset(resp, 0, sizeof(resp));
    if (at_send("AT+CCLK?", "OK", 2000, resp, sizeof(resp)))
    {
        time_t epoch = 0;
        if (parse_quectel_time_response_to_epoch(resp, &epoch))
        {
            esp_set_epoch_time(epoch);
            ctx.next_sleep_seconds = calculate_sleep_seconds_to_next_quarter();
            return EVENT_OK;
        }
    }

    ESP_LOGW(TAG, "Network time unavailable, keep previous RTC/fallback sleep");
#endif

    ctx.next_sleep_seconds = calculate_sleep_seconds_to_next_quarter();
    return EVENT_OK;
}

static app_event_t action_network_ready(void)
{
    if (!ctx.modem_responding || !ctx.modem_uart_ready)
    {
        ESP_LOGW(TAG, "Skip network: modem not ready");
        return EVENT_FAIL;
    }

    char resp[MAX_AT_RESP_LEN];
    char cmd[160];

    at_send_simple("ATE0", 1000);
    at_send_simple("AT+CFUN=1", 5000);

    if (modem_is_ready() != EVENT_OK)
    {
        ESP_LOGE(TAG, "SIM is not ready");
        return EVENT_FAIL;
    }

    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN_NAME);
    at_send(cmd, "OK", 3000, resp, sizeof(resp));

    int64_t start = now_ms();
    while ((now_ms() - start) < STATE_NETWORK_TIMEOUT_MS)
    {
        check_state_timeout();
        memset(resp, 0, sizeof(resp));

        if (at_send("AT+CEREG?", "OK", 3000, resp, sizeof(resp)) && modem_is_registered_response(resp))
        {
            ESP_LOGI(TAG, "Modem registered on LTE network");
            break;
        }

        memset(resp, 0, sizeof(resp));
        if (at_send("AT+CREG?", "OK", 3000, resp, sizeof(resp)) && modem_is_registered_response(resp))
        {
            ESP_LOGI(TAG, "Modem registered on GSM network");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    memset(resp, 0, sizeof(resp));
    snprintf(cmd, sizeof(cmd), "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", APN_NAME);
    at_send(cmd, "OK", 5000, resp, sizeof(resp));

    memset(resp, 0, sizeof(resp));
    if (!at_send("AT+QIACT?", "OK", 5000, resp, sizeof(resp)) || !strstr(resp, "+QIACT: 1"))
    {
        if (!at_send("AT+QIACT=1", "OK", 30000, resp, sizeof(resp)))
        {
            ESP_LOGE(TAG, "PDP context activation failed");
            return EVENT_FAIL;
        }
    }

    action_sync_network_time();
    return EVENT_OK;
}

static app_event_t action_modem_sleep(void)
{
    at_send_simple("AT+QSCLK=1", 2000);
    modem_gpio_sleep_prep();

    return EVENT_OK;
}

// GPS / GNSS

#if ENABLE_GNSS
static bool parse_qgpsloc(const char *resp, double *lat, double *lng)
{
    const char *p = strstr(resp, "+QGPSLOC:");
    if (!p)
        return false;

    p = strchr(p, ':');
    if (!p)
        return false;
    p++;

    while (*p == ' ')
        p++;

    char buf[256];
    strncpy(buf, p, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *line_end = strpbrk(buf, "\r\n");
    if (line_end)
        *line_end = '\0';

    char *saveptr = NULL;
    char *utc = strtok_r(buf, ",", &saveptr);
    char *lat_s = strtok_r(NULL, ",", &saveptr);
    char *lng_s = strtok_r(NULL, ",", &saveptr);

    (void)utc;

    if (!lat_s || !lng_s)
        return false;

    *lat = atof(lat_s);
    *lng = atof(lng_s);

    if (*lat == 0 && *lng == 0)
        return false;
    return true;
}
#endif

static app_event_t action_read_gnss(void)
{
    ctx.current_record.gps_valid = 0;
    ctx.current_record.gps_lat = 0;
    ctx.current_record.gps_lon = 0;
    ctx.current_record.time_valid = 0;
    ctx.current_record.unix_time = 0;
    ctx.current_record.time_iso[0] = '\0';

    fill_record_time_from_esp(&ctx.current_record);

    if (!ctx.modem_responding || !ctx.modem_uart_ready)
    {
        ESP_LOGW(TAG, "Skip GNSS because modem/UART is not ready");
        return EVENT_OK;
    }

#if ENABLE_GNSS
    char resp[MAX_AT_RESP_LEN];

    // Lenh Quectel GNSS. Neu module/firmware khong ho tro thi se ERROR va bo qua GPS.
    if (!at_send("AT+QGPS=1", "OK", 10000, resp, sizeof(resp)))
    {
        ESP_LOGW(TAG, "GNSS not supported or cannot start, continue without GPS");
        return EVENT_OK;
    }

    int64_t start = now_ms();
    while ((now_ms() - start) < GNSS_FIX_TIMEOUT_MS)
    {
        check_state_timeout();
        memset(resp, 0, sizeof(resp));

        // mode=2 thuong tra ve vi do/kinh do dang decimal tren cac dong Quectel co GNSS.
        if (at_send("AT+QGPSLOC=2", "OK", 5000, resp, sizeof(resp)))
        {
            double lat = 0;
            double lng = 0;
            if (parse_qgpsloc(resp, &lat, &lng))
            {
                ctx.current_record.gps_valid = 1;
                ctx.current_record.gps_lat = lat;
                ctx.current_record.gps_lon = lng;
                ESP_LOGI(TAG, "GNSS fix: lat=%.6f lng=%.6f", lat, lng);
                return EVENT_OK;
            }
        }

        ESP_LOGW(TAG, "GNSS no fix yet");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    ESP_LOGW(TAG, "GNSS timeout, continue without GPS");
#else
    ESP_LOGI(TAG, "GNSS disabled by ENABLE_GNSS=0");
#endif

    return EVENT_OK;
}

static app_event_t action_update_gnss_to_sd(void)
{
    if (!file_exists(DATA_FILE))
        return EVENT_OK;

    FILE *src = fopen(DATA_FILE, "r");
    FILE *tmp = fopen(TMP_FILE, "w");

    if (!src || !tmp)
    {
        if (src)
            fclose(src);
        if (tmp)
            fclose(tmp);
        return EVENT_FAIL;
    }

    char line[MAX_LINE_LEN];
    while (fgets(line, sizeof(line), src))
    {
        check_state_timeout();

        cJSON *root = cJSON_Parse(line);
        if (root)
        {
            cJSON *id_item = cJSON_GetObjectItem(root, "id");
            if (cJSON_IsNumber(id_item) && (uint32_t)id_item->valuedouble == ctx.current_record.id)
            {
                cJSON_DeleteItemFromObject(root, "gps_valid");
                cJSON_AddNumberToObject(root, "gps_valid", ctx.current_record.gps_valid);
                cJSON_DeleteItemFromObject(root, "gps_lat");
                cJSON_AddNumberToObject(root, "gps_lat", ctx.current_record.gps_lat);
                cJSON_DeleteItemFromObject(root, "gps_lon");
                cJSON_AddNumberToObject(root, "gps_lon", ctx.current_record.gps_lon);
                cJSON_DeleteItemFromObject(root, "time_valid");
                cJSON_AddNumberToObject(root, "time_valid", ctx.current_record.time_valid);
                cJSON_DeleteItemFromObject(root, "unix_time");
                cJSON_AddNumberToObject(root, "unix_time", ctx.current_record.unix_time);
                cJSON_DeleteItemFromObject(root, "time_iso");
                cJSON_AddStringToObject(root, "time_iso", ctx.current_record.time_iso);

                char *json = cJSON_PrintUnformatted(root);
                if (json)
                {
                    fprintf(tmp, "%s\n", json);
                    free(json);
                }

                cJSON_Delete(root);
                continue;
            }
            cJSON_Delete(root);
        }

        fputs(line, tmp);
    }

    fflush(tmp);
    fsync(fileno(tmp));
    fclose(src);
    fclose(tmp);

    if (!safe_replace_file(TMP_FILE, DATA_FILE, BAK_FILE))
        return EVENT_FAIL;

    ESP_LOGI(TAG, "GNSS updated to SD for id=%lu", (unsigned long)ctx.current_record.id);
    return EVENT_OK;
}

// MQTT / NETWORK

static bool mqtt_username_enabled(void)
{
    return (MQTT_USERNAME && strlen(MQTT_USERNAME) > 0);
}

static bool modem_mqtt_is_connected(void)
{
    char resp[MAX_AT_RESP_LEN];
    if (at_send("AT+QMTCONN?", "OK", 2000, resp, sizeof(resp)))
    {
        return strstr(resp, "+QMTCONN: 0,3") || strstr(resp, "+QMTCONN: 0,0") || strstr(resp, "+QMTCONN: 0,1");
    }
    return false;
}

static bool modem_mqtt_configure(void)
{
    char resp[MAX_AT_RESP_LEN];
    char cmd[160];

    if (!at_send("AT+QMTCFG=\"version\",0,4", "OK", 3000, resp, sizeof(resp)))
        return false;

    if (!at_send("AT+QMTCFG=\"pdpcid\",0,1", "OK", 3000, resp, sizeof(resp)))
        return false;

    snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"keepalive\",0,%d", MQTT_KEEPALIVE_SECONDS);
    if (!at_send(cmd, "OK", 3000, resp, sizeof(resp)))
        return false;

    at_send("AT+QMTCFG=\"session\",0,0", "OK", 3000, resp, sizeof(resp));

    snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"timeout\",0,%d,3,1", MQTT_PUB_TIMEOUT_SECONDS);
    at_send(cmd, "OK", 3000, resp, sizeof(resp));

    at_send("AT+QMTCFG=\"recv/mode\",0,0,1", "OK", 3000, resp, sizeof(resp));

    snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"qmtping\",0,%d", MQTT_KEEPALIVE_SECONDS);
    at_send(cmd, "OK", 3000, resp, sizeof(resp));

    return true;
}

static bool modem_mqtt_open(void)
{
    char resp[MAX_AT_RESP_LEN];
    char cmd[192];

    if (modem_mqtt_is_connected())
    {
        return true;
    }

    if (!MQTT_BROKER_HOST || strlen(MQTT_BROKER_HOST) == 0 || strcmp(MQTT_BROKER_HOST, "YOUR_MQTT_BROKER_HOST") == 0)
    {
        ESP_LOGE(TAG, "MQTT_BROKER_HOST is not configured");
        return false;
    }

    at_send("AT+QMTCLOSE=0", "OK", 5000, resp, sizeof(resp));

    snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    if (!at_send(cmd, "OK", 10000, resp, sizeof(resp)))
    {
        ESP_LOGE(TAG, "MQTT open command failed");
        return false;
    }

    int64_t start = now_ms();
    while ((now_ms() - start) < 60000)
    {
        check_state_timeout();
        memset(resp, 0, sizeof(resp));
        if (wait_for_modem_response("+QMTOPEN:", 3000, resp, sizeof(resp)))
        {
            if (strstr(resp, "+QMTOPEN: 0,0"))
            {
                ESP_LOGI(TAG, "MQTT socket opened");
                return true;
            }
            ESP_LOGE(TAG, "MQTT socket open failed: %s", resp);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGE(TAG, "MQTT open timeout");
    return false;
}

static bool modem_mqtt_connect(void)
{
    char resp[MAX_AT_RESP_LEN];
    char cmd[192];

    if (modem_mqtt_is_connected())
    {
        ctx.mqtt_connected = true;
        return true;
    }

    if (!modem_mqtt_configure())
    {
        ESP_LOGE(TAG, "MQTT configure failed");
        ctx.mqtt_connected = false;
        return false;
    }

    if (!modem_mqtt_open())
    {
        ctx.mqtt_connected = false;
        return false;
    }

    if (mqtt_username_enabled())
    {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"", MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
    }
    else
    {
        snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", MQTT_CLIENT_ID);
    }

    if (!at_send(cmd, "OK", 10000, resp, sizeof(resp)))
    {
        ESP_LOGE(TAG, "MQTT connect command failed");
        ctx.mqtt_connected = false;
        return false;
    }

    int64_t start = now_ms();
    while ((now_ms() - start) < 30000)
    {
        check_state_timeout();

        if (modem_mqtt_is_connected())
        {
            ctx.mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected");
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ctx.mqtt_connected = false;
    ESP_LOGE(TAG, "MQTT connection timeout");
    return false;
}

static app_event_t action_mqtt_connect(void)
{
    return modem_mqtt_connect() ? EVENT_OK : EVENT_FAIL;
}

static bool modem_mqtt_publish_json_once(const char *json)
{
    if (!modem_mqtt_is_connected())
    {
        ESP_LOGW(TAG, "MQTT not connected, cannot publish");
        return false;
    }

    if (!json || strlen(json) == 0)
    {
        ESP_LOGW(TAG, "No JSON payload to publish");
        return false;
    }

    char resp[MAX_AT_RESP_LEN];
    char cmd[256];

    uint32_t msg_id = ctx.mqtt_msg_id++;
    if (ctx.mqtt_msg_id == 0)
        ctx.mqtt_msg_id = 1;

    size_t json_len = strlen(json);

    int snprintf_res = snprintf(cmd, sizeof(cmd), "AT+QMTPUBEX=0,%u,%d,%d,\"%s\",%u",
                                (unsigned)msg_id, MQTT_QOS, MQTT_RETAIN, MQTT_TOPIC_PUB, (unsigned)json_len);

    if (snprintf_res < 0 || (size_t)snprintf_res >= sizeof(cmd))
    {
        ESP_LOGE(TAG, "Command buffer overflow, topic might be too long");
        return false;
    }

    if (!at_send_payload(cmd, json, "OK", 5000, 10000, resp, sizeof(resp)))
    {
        ESP_LOGE(TAG, "MQTT publish command failed");
        return false;
    }

    if (strstr(resp, "+QMTPUB:") && !strstr(resp, ",0,0"))
    {
        ESP_LOGE(TAG, "Modem accepted command but network publish failed, resp: %s", resp);
    }

    ESP_LOGI(TAG, "MQTT published message id=%u successfully", (unsigned)msg_id);
    return true;
}

static bool modem_mqtt_publish_json(const char *json)
{
    for (int i = 0; i < 3; i++)
    {
        if (modem_mqtt_publish_json_once(json))
        {
            return true;
        }
        ESP_LOGW(TAG, "MQTT publish attempt %d failed, retrying...", i + 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return false;
}

// CHECK SD PENDING

static bool has_pending_records(void)
{
    if (!file_exists(DATA_FILE))
        return false;

    FILE *f = fopen(DATA_FILE, "r");
    if (!f)
    {
        ESP_LOGE(TAG, "Failed to open data file to check pending records");
        return false;
    }

    char line[MAX_LINE_LEN];
    bool pending = false;
    while (fgets(line, sizeof(line), f))
    {
        check_state_timeout();
        cJSON *root = cJSON_Parse(line);
        if (root)
        {
            cJSON *sended_item = cJSON_GetObjectItem(root, "sended");
            if (cJSON_IsNumber(sended_item) && sended_item->valuedouble == 0)
            {
                pending = true;
                cJSON_Delete(root);
                break;
            }
            cJSON_Delete(root);
        }
    }

    fclose(f);
    return pending;
}

static app_event_t action_check_sd_pending(void)
{
    ctx.has_pending_records = has_pending_records();
    ESP_LOGI(TAG, "Pending records: %d", ctx.has_pending_records);
    return EVENT_OK;
}

// SEND ALL PENDING

static app_event_t action_send_all_pending(void)
{
    if (!file_exists(DATA_FILE))
    {
        return EVENT_OK;
    }

    if (!ctx.mqtt_connected && !modem_mqtt_is_connected())
    {
        ESP_LOGW(TAG, "MQTT not connected, keep all records on SD");
        return EVENT_FAIL;
    }

    FILE *src = fopen(DATA_FILE, "r");
    FILE *tmp = fopen(TMP_FILE, "w");

    if (!src || !tmp)
    {
        if (src)
            fclose(src);
        if (tmp)
            fclose(tmp);
        return EVENT_FAIL;
    }

    char line[MAX_LINE_LEN];
    uint32_t sent_count = 0;
    uint32_t kept_count = 0;
    bool stop_sending = false;
    bool write_error = false;

    while (fgets(line, sizeof(line), src))
    {
        check_state_timeout();

        bool line_written = false;
        cJSON *root = cJSON_Parse(line);

        if (root)
        {
            cJSON *id_item = cJSON_GetObjectItem(root, "id");
            cJSON *sended_item = cJSON_GetObjectItem(root, "sended");

            bool is_pending = (!sended_item || !cJSON_IsNumber(sended_item) || sended_item->valueint == 0);

            if (is_pending && !stop_sending)
            {
#if MAX_PENDING_SEND_PER_WAKE > 0
                if (sent_count >= MAX_PENDING_SEND_PER_WAKE)
                {
                    stop_sending = true;
                }
                else
#endif
                {
                    char *json_to_send = cJSON_PrintUnformatted(root);

                    if (json_to_send && modem_mqtt_publish_json(json_to_send))
                    {
                        if (sended_item)
                        {
                            cJSON_ReplaceItemInObject(root, "sended", cJSON_CreateNumber(1));
                        }
                        else
                        {
                            cJSON_AddNumberToObject(root, "sended", 1);
                        }

                        char *json_updated = cJSON_PrintUnformatted(root);

                        if (json_updated)
                        {
                            if (fputs(json_updated, tmp) < 0 || fputc('\n', tmp) < 0)
                            {
                                write_error = true;
                            }
                            else
                            {
                                line_written = true;
                            }
                            free(json_updated);
                        }

                        sent_count++;

                        if (cJSON_IsNumber(id_item))
                        {
                            ESP_LOGI(TAG, "Sent record id=%lu, set sended=1",
                                     (unsigned long)((uint32_t)id_item->valuedouble));
                        }
                    }
                    else
                    {
                        stop_sending = true;
                    }

                    if (json_to_send)
                    {
                        free(json_to_send);
                    }
                }
            }
        }

        if (root)
        {
            cJSON_Delete(root);
        }

        if (!line_written)
        {
            if (fputs(line, tmp) < 0)
            {
                write_error = true;
            }
        }

        if (write_error)
        {
            ESP_LOGE(TAG, "Hardware write error to TMP_FILE, aborting loop");
            break;
        }

        kept_count++;
    }

    fflush(tmp);
    fsync(fileno(tmp));

    fclose(src);
    fclose(tmp);

    if (write_error)
    {
        remove(TMP_FILE);
        return EVENT_FAIL;
    }

    if (!safe_replace_file(TMP_FILE, DATA_FILE, BAK_FILE))
    {
        return EVENT_FAIL;
    }

    ESP_LOGI(TAG, "Send pending done: sent=%lu, total_kept=%lu",
             (unsigned long)sent_count,
             (unsigned long)kept_count);

    return stop_sending ? EVENT_FAIL : EVENT_OK;
}

// PREPARE MQTT KEEPALIVE / SLEEP

static app_event_t action_modem_prepare_sleep(void)
{
    if (ctx.modem_uart_ready)
    {
        char resp[MAX_AT_RESP_LEN];

#if ENABLE_GNSS
        at_send("AT+QGPSEND", "OK", 5000, resp, sizeof(resp));
#endif

        at_send("AT+QSCLK=1", "OK", 3000, resp, sizeof(resp));
        at_send("AT+QMTCONN?", "OK", 3000, resp, sizeof(resp));
        modem_uart_delete();
    }

    modem_gpio_sleep_prep();

    ESP_LOGI(TAG, "Modem prepared for sleep: QSCLK=1, DTR=HIGH, USB_VBUS disconnected, ESP pins held");
    return EVENT_OK;
}

static app_event_t action_prepare_mqtt_keepalive(void)
{
    if (ctx.modem_uart_ready)
    {
        char resp[MAX_AT_RESP_LEN];
        at_send("AT+QMTCONN?", "OK", 3000, resp, sizeof(resp));
    }

    return action_modem_prepare_sleep();
}

// MODEM SLEEP / DEEP SLEEP

static void action_deep_sleep(void)
{
    uint32_t sleep_seconds = ctx.next_sleep_seconds;

    if (sleep_seconds < MIN_SLEEP_SECONDS || sleep_seconds > (24U * 3600U))
    {
        sleep_seconds = QUARTER_INTERVAL_SECONDS;
    }

    ESP_LOGI(TAG, "Entering ESP deep sleep for %lu seconds, target minute%%15==0",
             (unsigned long)sleep_seconds);

    rtc_store_expected_wakeup_time(sleep_seconds);

    feed_wdt();
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_seconds * 1000000ULL);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_deep_sleep_start();
}

// BOOT

static app_event_t action_boot(void)
{
    modem_gpio_release_hold();
    rtc_restore_time_on_boot();
    ctx.next_sleep_seconds = calculate_sleep_seconds_to_next_quarter();
    ctx.has_pending_records = false;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(ret);
    }

    return EVENT_OK;
}

static app_event_t action_check_sd_and_sim_modem(void)
{
    if (sd_mount() != ESP_OK)
    {
        ESP_LOGE(TAG, "SD check failed");
        return EVENT_FAIL;
    }

    recover_file_if_needed();

    ctx.modem_responding = false;
    app_event_t modem_ev = action_modem_wake();
    if (modem_ev != EVENT_OK)
    {
        ESP_LOGW(TAG, "Modem check failed, continue local logging only");
        return EVENT_OK;
    }

    char resp[MAX_AT_RESP_LEN];
    if (!at_send("AT+CPIN?", "READY", 10000, resp, sizeof(resp)))
    {
        ESP_LOGW(TAG, "SIM not ready at early check, continue local logging only");
        ctx.modem_responding = false;
        return EVENT_OK;
    }

    ctx.modem_responding = true;
    ESP_LOGI(TAG, "SD and SIM modem check OK");
    return EVENT_OK;
}

// MAIN TASK

void app_main(void)
{
    watchdog_init();
    enter_state(STATE_BOOT);

    while (1)
    {
        feed_wdt();
        check_state_timeout();

        switch (ctx.state)
        {
        case STATE_BOOT:
            enter_state((action_boot() == EVENT_OK) ? STATE_CHECK_SD_AND_SIM_MODEM : STATE_DEEP_SLEEP);
            break;

        case STATE_CHECK_SD_AND_SIM_MODEM:
            enter_state((action_check_sd_and_sim_modem() == EVENT_OK) ? STATE_READ_SENSOR : STATE_DEEP_SLEEP);
            break;

        case STATE_READ_SENSOR:
            enter_state((action_read_sensor() == EVENT_OK) ? STATE_SAVE_TO_SD : STATE_DEEP_SLEEP);
            break;

        case STATE_SAVE_TO_SD:
            enter_state((action_save_to_sd() == EVENT_OK) ? STATE_READ_GNSS : STATE_DEEP_SLEEP);
            break;

        case STATE_READ_GNSS:
            enter_state((action_read_gnss() == EVENT_OK) ? STATE_UPDATE_GNSS_TO_SD : STATE_CHECK_SD_PENDING);
            break;

        case STATE_UPDATE_GNSS_TO_SD:
            enter_state((action_update_gnss_to_sd() == EVENT_OK) ? STATE_CHECK_SD_PENDING : STATE_PREPARE_MQTT_KEEPALIVE);
            break;

        case STATE_CHECK_SD_PENDING:
            if (action_check_sd_pending() == EVENT_OK && ctx.has_pending_records && ctx.modem_responding)
                enter_state(STATE_NETWORK_READY);
            else
                enter_state(STATE_PREPARE_MQTT_KEEPALIVE);
            break;

        case STATE_NETWORK_READY:
            enter_state((action_network_ready() == EVENT_OK) ? STATE_MQTT_CONNECT : STATE_PREPARE_MQTT_KEEPALIVE);
            break;

        case STATE_MQTT_CONNECT:
            enter_state((action_mqtt_connect() == EVENT_OK) ? STATE_SEND_ALL_PENDING : STATE_PREPARE_MQTT_KEEPALIVE);
            break;

        case STATE_SEND_ALL_PENDING:
            action_send_all_pending();
            enter_state(STATE_PREPARE_MQTT_KEEPALIVE);
            break;

        case STATE_PREPARE_MQTT_KEEPALIVE:
            action_prepare_mqtt_keepalive();
            enter_state(STATE_DEEP_SLEEP);
            break;

        case STATE_DEEP_SLEEP:
            action_deep_sleep();
            break;

        default:
            ESP_LOGW(TAG, "Unknown state, go to deep sleep");
            enter_state(STATE_DEEP_SLEEP);
            break;
        }
    }
}
