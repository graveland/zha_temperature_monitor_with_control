#include "main.h"

#include "esp_check.h"
#include "string.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zboss_api.h"

#include "driver/gpio.h"
#include "led_strip.h"

#include "temp_sensor.h"
#include "nvs_functions.h"


static const char *TAG = "ESP_ZB_TEMP_SENSOR";
static led_strip_handle_t led_strip = NULL;
uint8_t read_failures = 0;

uint8_t ds18b20_device_num = 0;
ds18b20_device_handle_t ds18b20s[HA_ESP_NUM_T_SENSORS];
uint8_t ep_to_ds[HA_ESP_NUM_T_SENSORS]; // Use 0xff to indicate no link

// Forward declarations
static void led_identify_blink(uint16_t identify_time);
static void query_bound_switch_state(void);

// Track the current state of the controlled switch
static bool switch_state = false;
static bool startup_temp_received = false;  // Block switch ops until first valid temp

// Temperature watchdog - detect stuck/missing sensor readings
static float last_fed_temp = DSB1820_BAD_TEMP;
static int64_t last_fed_time = 0;  // 0 = never fed
static float last_temp_reading = DSB1820_BAD_TEMP;
static int64_t last_temp_change_time = 0;

// Network watchdog - restart, never wipe, a device that fails to come back
// onto its network. Only a long run of failed restarts falls back to a
// factory reset, which erases the bindings.
#define WATCHDOG_RESTART_AFTER_MINUTES 5
#define WATCHDOG_FACTORY_RESET_AFTER_RESTARTS 12
static int network_not_joined_count = 0;
static bool watchdog_restarts_cleared = false;

// Latest ambient reading received from a bound Temperature Measurement server
static float remote_temp = DSB1820_BAD_TEMP;
static int64_t remote_temp_time = 0;  // 0 = never received

// Configurable temperature thresholds (loaded from NVS or defaults)
static float temp_control_on_threshold = TEMP_CONTROL_ON_THRESHOLD;
static float temp_control_off_threshold = TEMP_CONTROL_OFF_THRESHOLD;

// Zigbee attribute values (must be static/global for ESP Zigbee stack)
static int16_t zb_on_threshold_attr = 0;
static int16_t zb_off_threshold_attr = 0;
static int16_t zb_remote_temp_attr = ZB_TEMP_MEASUREMENT_UNKNOWN;
static uint16_t zb_reset_count = 0;
static bool reset_count_reported = false;  // the one-shot reset count report goes out once per boot

// Diagnostics cluster attributes, attribute id = index. The stack keeps its own copy of each value, so
// diag_publish() writes them in; the counters here are plain variables.
static uint16_t diag_raw_total = 0;         // ZCL frames handed to the raw handler
static uint16_t diag_raw_reports = 0;       // Report Attributes frames for Temperature Measurement
static uint16_t diag_raw_report_src = 0;    // short address of the sender of the last such frame
static uint16_t diag_report_cb = 0;         // report callbacks delivered to the application, any cluster
static uint16_t diag_report_cb_status = 0;  // status of the last report callback
static uint16_t diag_report_stored = 0;     // reports accepted and mirrored as outdoor temperature
static uint16_t diag_other_cb = 0;          // action callbacks with no handler
static uint16_t diag_other_cb_id = 0;       // id of the last such callback
static uint16_t diag_reset_reason = 0;      // esp_reset_reason() of this boot
static uint16_t diag_crumb_zb_prev = 0;     // last Zigbee callback breadcrumb before the previous reset
static uint16_t diag_crumb_timer_prev = 0;  // last esp_timer callback breadcrumb before the previous reset
static uint16_t diag_min_free_heap_kb = 0;  // lowest free heap since boot
static uint16_t diag_stack_free_zb = 0;     // lowest free bytes of the Zigbee task stack
static uint16_t diag_stack_free_timer = 0;  // lowest free bytes of the esp_timer task stack
static uint16_t diag_restart_cause_prev = 0;  // why the previous run called esp_restart(), 0 if it did not

static uint16_t *const diag_values[] = {
    &diag_raw_total, &diag_raw_reports, &diag_raw_report_src, &diag_report_cb,
    &diag_report_cb_status, &diag_report_stored, &diag_other_cb, &diag_other_cb_id,
    &diag_reset_reason, &diag_crumb_zb_prev, &diag_crumb_timer_prev,
    &diag_min_free_heap_kb, &diag_stack_free_zb, &diag_stack_free_timer, &diag_restart_cause_prev,
};
#define DIAG_COUNT (sizeof(diag_values) / sizeof(diag_values[0]))

static TaskHandle_t zb_task_handle;

// Breadcrumbs: the last step each context reached, in memory that survives a software, watchdog or panic
// reset, so the next boot can report where the previous run stopped.
#define CRUMB_MAGIC 0xC0FFEE15
static RTC_NOINIT_ATTR uint32_t rtc_crumb_magic;
static RTC_NOINIT_ATTR uint16_t rtc_crumb_zb;
static RTC_NOINIT_ATTR uint16_t rtc_crumb_timer;
static RTC_NOINIT_ATTR uint16_t rtc_restart_cause;

enum {
    CRUMB_ZB_RAW = 1,           // raw command handler
    CRUMB_ZB_REPORT_IN = 2,     // report callback entered
    CRUMB_ZB_REPORT_OK = 3,     // report validated
    CRUMB_ZB_REPORT_STORED = 4, // received value stored for the timer to publish
    CRUMB_ZB_REPORT_DONE = 5,   // report callback finished
    CRUMB_ZB_READ_RESP = 7,     // read attribute response handler
    CRUMB_ZB_SET_CB = 8,        // attribute write handler
    CRUMB_ZB_OTHER_CB = 9,      // unhandled action callback
    CRUMB_ZB_SIGNAL = 10,       // app signal handler
};
enum {
    CRUMB_TIMER_TEMP = 20,      // temperature timer entered
    CRUMB_TIMER_READ = 21,      // sensors read
    CRUMB_TIMER_CONTROL = 22,   // switch control
    CRUMB_TIMER_REPORT = 23,    // reporting the local temperature
    CRUMB_TIMER_DIAG = 24,      // publishing diagnostics
    CRUMB_TIMER_DONE = 25,      // temperature timer finished
    CRUMB_TIMER_REMOTE = 26,    // publishing the received outdoor temperature
    CRUMB_TIMER_WATCHDOG = 30,  // watchdog timer entered
    CRUMB_TIMER_SYNC = 40,      // switch sync timer entered
};
enum {
    RESTART_NO_READINGS = 1,    // watchdog: no valid temperature reading
    RESTART_STUCK_TEMP = 2,     // watchdog: temperature stuck
    RESTART_NOT_JOINED = 3,     // watchdog: not joined to the network
    RESTART_REBOOT_SWITCH = 4,  // On written to the reboot switch endpoint
    RESTART_OTA_DONE = 5,       // OTA image applied
    RESTART_OTHER = 99,         // esp_restart() from code that does not label it
};
#define CRUMB_ZB(step) (rtc_crumb_zb = (step))
#define CRUMB_TIMER(step) (rtc_crumb_timer = (step))

// Remember why the firmware restarts itself, so the next boot can report it
static void restart_for(uint16_t cause)
{
    rtc_restart_cause = cause;
    esp_restart();
}

static void restart_shutdown_handler(void)
{
    if (rtc_restart_cause == 0) {
        rtc_restart_cause = RESTART_OTHER;
    }
}

// A received temperature waits here for the timer to publish it, so no ZCL send happens inside the report callback
static volatile bool remote_temp_pending = false;

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

// Feed watchdog - just record temp and time, no decisions
static void feed_watchdog(float temp)
{
    if (temp == DSB1820_BAD_TEMP) {
        return;
    }
    last_fed_temp = temp;
    last_fed_time = esp_timer_get_time();
}

// Watchdog timer callback - does all the decision logic
/* Remote rejoin (endpoint 11): state and forward declaration. */
static uint16_t zb_rejoin_count = 0;
static void rejoin_request_async(const char *reason);

static void temp_timer_callback(void *arg);
static void watchdog_timer_callback(void *arg)
{
    CRUMB_TIMER(CRUMB_TIMER_WATCHDOG);
    int64_t now = esp_timer_get_time();

    // Check if we've been fed recently with a valid temp
    if (last_fed_time == 0 || last_fed_temp == DSB1820_BAD_TEMP) {
        // Never fed or last feed was bad
        int64_t elapsed;
        if (last_fed_time == 0) {
            elapsed = now;  // time since boot if never fed
        } else {
            elapsed = now - last_fed_time;  // time since last (bad) feed
        }

        int elapsed_minutes = (int)(elapsed / 60000000);  // Convert microseconds to minutes
        ESP_LOGW(TAG, "Watchdog: No valid temperature reading for %d minute(s) (timeout at %d minutes)",
                 elapsed_minutes, TEMP_WATCHDOG_TIMEOUT_MINUTES);

        if (elapsed >= TEMP_WATCHDOG_TIMEOUT_US) {
            ESP_LOGW(TAG, "Watchdog: No valid readings for %d minutes - rebooting",
                     TEMP_WATCHDOG_TIMEOUT_MINUTES);
            restart_for(RESTART_NO_READINGS);
        }
        return;
    }

    // Have valid readings - check for stuck temperature
    float temp_diff = last_fed_temp - last_temp_reading;
    if (temp_diff < 0) temp_diff = -temp_diff;

    if (temp_diff >= TEMP_CHANGE_THRESHOLD) {
        // Temperature changed - update tracking
        last_temp_reading = last_fed_temp;
        last_temp_change_time = now;
        ESP_LOGD(TAG, "Watchdog: temp changed to %.2f°C", last_fed_temp);
    } else if (last_temp_change_time != 0) {
        // Check for stuck value
        int64_t elapsed = now - last_temp_change_time;
        if (elapsed >= TEMP_WATCHDOG_TIMEOUT_US) {
            ESP_LOGW(TAG, "Watchdog: Temperature stuck at %.2f°C for %d minutes - rebooting",
                     last_fed_temp, TEMP_WATCHDOG_TIMEOUT_MINUTES);
            restart_for(RESTART_STUCK_TEMP);
        }
    } else {
        // First valid reading - start tracking
        last_temp_reading = last_fed_temp;
        last_temp_change_time = now;
        ESP_LOGI(TAG, "Watchdog initialized: %.2f°C", last_fed_temp);
    }

    // Network watchdog - restart if not joined for too long
    esp_zb_lock_acquire(portMAX_DELAY);
    bool joined = esp_zb_bdb_dev_joined();
    bool factory_new = esp_zb_bdb_is_factory_new();
    esp_zb_lock_release();

    if (joined) {
        network_not_joined_count = 0;
        if (!watchdog_restarts_cleared) {
            clear_watchdog_restarts();
            watchdog_restarts_cleared = true;
        }
        return;
    }

    if (factory_new) {
        // Network steering retries itself and there is no state left to protect.
        return;
    }

    network_not_joined_count++;
    ESP_LOGW(TAG, "Watchdog: Not joined to network (count: %d)", network_not_joined_count);
    if (network_not_joined_count > WATCHDOG_RESTART_AFTER_MINUTES) {
        uint8_t restarts = increment_watchdog_restarts();
        if (restarts >= WATCHDOG_FACTORY_RESET_AFTER_RESTARTS) {
            ESP_LOGW(TAG, "Watchdog: %d restarts without rejoining - factory reset", restarts);
            clear_watchdog_restarts();
            esp_zb_factory_reset();
        }
        ESP_LOGW(TAG, "Watchdog: Not joined to network for %d minutes - restarting", network_not_joined_count);
        restart_for(RESTART_NOT_JOINED);
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    CRUMB_ZB(CRUMB_ZB_SET_CB);
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Cluster attr write: cluster=0x%x, attr=0x%x",
             message->info.cluster, message->attribute.id);

    // Handle Identify cluster
    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY) {
        // Handle identify command - attribute 0x0 is identify_time
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
            uint16_t identify_time = *(uint16_t *)message->attribute.data.value;
            ESP_LOGI(TAG, "Identify command received, time=%d seconds", identify_time);
            if (identify_time > 0) {
                led_identify_blink(identify_time);
            }
        }
    }
    // Handle writes to Thermostat cluster for temperature thresholds
    else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT &&
        message->info.dst_endpoint >= HA_ESP_TEMP_START_ENDPOINT &&
        message->info.dst_endpoint < (HA_ESP_TEMP_START_ENDPOINT + HA_ESP_NUM_T_SENSORS))
    {
        // occupied_heating_setpoint (0x0012) = ON threshold
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID)
        {
            int16_t new_value = *(int16_t *)message->attribute.data.value;
            float new_threshold = (float)new_value / 100.0f;

            ESP_LOGI(TAG, "Received new ON threshold (heating setpoint): %.2f°C", new_threshold);

            // Update the runtime variable and Zigbee attribute
            temp_control_on_threshold = new_threshold;
            zb_on_threshold_attr = new_value;

            // Save to NVS
            if (set_temp_thresholds(temp_control_on_threshold, temp_control_off_threshold) == ESP_OK)
            {
                ESP_LOGI(TAG, "ON threshold updated and saved to NVS");
            }
            else
            {
                ESP_LOGW(TAG, "Failed to save ON threshold to NVS");
            }
        }
        // occupied_cooling_setpoint (0x0011) = OFF threshold
        else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID)
        {
            int16_t new_value = *(int16_t *)message->attribute.data.value;
            float new_threshold = (float)new_value / 100.0f;

            ESP_LOGI(TAG, "Received new OFF threshold (cooling setpoint): %.2f°C", new_threshold);

            // Update the runtime variable and Zigbee attribute
            temp_control_off_threshold = new_threshold;
            zb_off_threshold_attr = new_value;

            // Save to NVS
            if (set_temp_thresholds(temp_control_on_threshold, temp_control_off_threshold) == ESP_OK)
            {
                ESP_LOGI(TAG, "OFF threshold updated and saved to NVS");
            }
            else
            {
                ESP_LOGW(TAG, "Failed to save OFF threshold to NVS");
            }
        }
    }
    // Handle writes to On/Off cluster on reboot endpoint
    // Remote rejoin switch (stateless: any write to onOff triggers).
    else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
             message->info.dst_endpoint == HA_ESP_REJOIN_ENDPOINT)
    {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
        {
            ESP_LOGW(TAG, "Rejoin switch triggered");
            rejoin_request_async("remote switch");
        }
        return ret;
    }
    else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
             message->info.dst_endpoint == HA_ESP_REBOOT_ENDPOINT)
    {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
        {
            bool on_off_value = *(bool *)message->attribute.data.value;
            if (on_off_value) {
                ESP_LOGW(TAG, "Reboot switch triggered - rebooting device");
                // Short delay to allow Zigbee response to be sent
                vTaskDelay(pdMS_TO_TICKS(500));
                restart_for(RESTART_REBOOT_SWITCH);
            }
        }
    }

    return ret;
}

static esp_err_t zb_identify_handler(esp_zb_zcl_identify_effect_message_t *message) {
    ESP_LOGI(TAG, "Identify effect command received: effect_id=%d, effect_variant=%d",
             message->effect_id, message->effect_variant);

    // Blink LED for identify - use 5 seconds as default
    led_identify_blink(5);

    return ESP_OK;
}

// OTA partition and handle (must be static for persistence across callbacks)
static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;
static bool s_tagid_received = false;

static esp_err_t esp_element_ota_data(uint32_t total_size, const void *payload, uint16_t payload_size, void **outbuf, uint16_t *outlen)
{
    static uint16_t tagid = 0;
    void *data_buf = NULL;
    uint16_t data_len;

    if (!s_tagid_received) {
        uint32_t length = 0;
        if (!payload || payload_size <= OTA_ELEMENT_HEADER_LEN) {
            ESP_LOGE(TAG, "Invalid element format");
            return ESP_ERR_INVALID_ARG;
        }

        tagid  = *(const uint16_t *)payload;
        length = *(const uint32_t *)(payload + sizeof(tagid));
        if ((length + OTA_ELEMENT_HEADER_LEN) != total_size) {
            ESP_LOGE(TAG, "Invalid element length [%ld/%ld]", length, total_size);
            return ESP_ERR_INVALID_ARG;
        }

        s_tagid_received = true;

        data_buf = (void *)(payload + OTA_ELEMENT_HEADER_LEN);
        data_len = payload_size - OTA_ELEMENT_HEADER_LEN;
    } else {
        data_buf = (void *)payload;
        data_len = payload_size;
    }

    switch (tagid) {
        case UPGRADE_IMAGE:
            *outbuf = data_buf;
            *outlen = data_len;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported element tag identifier %d", tagid);
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t zb_ota_upgrade_status_handler(esp_zb_zcl_ota_upgrade_value_message_t message)
{
    static uint32_t total_size = 0;
    static uint32_t offset = 0;
    static int64_t start_time = 0;
    esp_err_t ret = ESP_OK;

    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        switch (message.upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ESP_LOGI(TAG, "-- OTA upgrade start");

            // Clean up any previous incomplete OTA session
            if (s_ota_handle != 0) {
                ESP_LOGW(TAG, "Cleaning up previous OTA handle");
                esp_ota_end(s_ota_handle);
                s_ota_handle = 0;
            }
            offset = 0;
            total_size = 0;
            s_tagid_received = false;

            start_time = esp_timer_get_time();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            if (!s_ota_partition) {
                ESP_LOGE(TAG, "Failed to find OTA partition");
                return ESP_FAIL;
            }
            ret = esp_ota_begin(s_ota_partition, 0, &s_ota_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to begin OTA partition: %s", esp_err_to_name(ret));
                return ret;
            }
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            total_size = message.ota_header.image_size;
            offset += message.payload_size;
            ESP_LOGI(TAG, "-- OTA Client receives data: progress [%ld/%ld]", offset, total_size);
            if (message.payload_size && message.payload) {
                uint16_t payload_size = 0;
                void *payload = NULL;
                ret = esp_element_ota_data(total_size, message.payload, message.payload_size, &payload, &payload_size);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to element OTA data: %s", esp_err_to_name(ret));
                    return ret;
                }
                ret = esp_ota_write(s_ota_handle, (const void *)payload, payload_size);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to write OTA data to partition: %s", esp_err_to_name(ret));
                    return ret;
                }
            }
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "-- OTA upgrade apply");
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            ret = offset == total_size ? ESP_OK : ESP_FAIL;
            offset = 0;
            total_size = 0;
            s_tagid_received = false;
            ESP_LOGI(TAG, "-- OTA upgrade check status: %s", esp_err_to_name(ret));
            break;

        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            ESP_LOGI(TAG, "-- OTA Finish");
            ESP_LOGI(TAG, "-- OTA Information: version: 0x%lx, manufacturer code: 0x%x, image type: 0x%x, total size: %ld bytes, cost time: %lld ms",
                     message.ota_header.file_version, message.ota_header.manufacturer_code, message.ota_header.image_type,
                     message.ota_header.image_size, (esp_timer_get_time() - start_time) / 1000);
            ret = esp_ota_end(s_ota_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to end OTA partition: %s", esp_err_to_name(ret));
                return ret;
            }
            ret = esp_ota_set_boot_partition(s_ota_partition);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set OTA boot partition: %s", esp_err_to_name(ret));
                return ret;
            }
            ESP_LOGW(TAG, "Prepare to restart system");
            restart_for(RESTART_OTA_DONE);
            break;

        default:
            ESP_LOGI(TAG, "OTA status: %d", message.upgrade_status);
            break;
        }
    }
    return ret;
}

static esp_err_t zb_ota_upgrade_query_image_resp_handler(esp_zb_zcl_ota_upgrade_query_image_resp_message_t message)
{
    ESP_LOGI(TAG, "OTA query response: status(%d), size(%lu), version(0x%lx), type(0x%x), manufacturer(0x%x)",
             message.query_status, message.image_size, message.file_version,
             message.image_type, message.manufacturer_code);

    if (message.query_status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "OTA image available, upgrade will start automatically");
        return ESP_OK;
    } else {
        ESP_LOGI(TAG, "No OTA image available");
        return ESP_FAIL;
    }
}

static void init_rgb_led(void) {
    // Configure LED strip (WS2812 or similar RGB LED on GPIO8)
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1, // Single RGB LED
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // Turn off LED initially
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "RGB LED initialized on GPIO %d", RGB_LED_GPIO);
}

static void set_led_color(uint8_t red, uint8_t green, uint8_t blue) {
    if (led_strip) {
        led_strip_set_pixel(led_strip, 0, red, green, blue);
        led_strip_refresh(led_strip);
    }
}

static void led_identify_blink(uint16_t identify_time) {
    // Blink BLUE for identify command
    ESP_LOGI(TAG, "Identify requested for %d seconds - blinking BLUE", identify_time);

    uint16_t blink_count = identify_time * 2; // Blink twice per second
    for (uint16_t i = 0; i < blink_count; i++) {
        set_led_color(0, 0, 255); // Blue
        vTaskDelay(pdMS_TO_TICKS(250));
        set_led_color(0, 0, 0); // Off
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    // Turn off LED after identify
    set_led_color(0, 0, 0);
}

static void switch_sync_timer_callback(void *arg)
{
    CRUMB_TIMER(CRUMB_TIMER_SYNC);
    query_bound_switch_state();
}

static void start_temp_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &temp_timer_callback,
        .name = "temp_timer"};

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, ESP_TEMP_SENSOR_UPDATE_INTERVAL));

    // Watchdog timer - runs every minute to check for stuck/missing sensor readings
    const esp_timer_create_args_t watchdog_timer_args = {
        .callback = &watchdog_timer_callback,
        .name = "watchdog_timer"};

    esp_timer_handle_t watchdog_timer;
    ESP_ERROR_CHECK(esp_timer_create(&watchdog_timer_args, &watchdog_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(watchdog_timer, 60 * 1000000));  // 1 minute

    // Switch state sync timer - runs every 5 minutes to verify/correct bound switch state
    const esp_timer_create_args_t switch_sync_timer_args = {
        .callback = &switch_sync_timer_callback,
        .name = "switch_sync_timer"};

    esp_timer_handle_t switch_sync_timer;
    ESP_ERROR_CHECK(esp_timer_create(&switch_sync_timer_args, &switch_sync_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(switch_sync_timer, 5 * 60 * 1000000));  // 5 minutes

    // Query switch state immediately on startup
    query_bound_switch_state();
}



static void report_attr(uint8_t ep, uint16_t cluster_id, uint16_t attribute_id)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = attribute_id;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = cluster_id;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = ep;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGD(TAG, "Send 'report attributes' command");
}

void report_temp_attr(uint8_t ep)
{
    report_attr(ep, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID);
}

// Sent from the temperature timer, once per boot, after the first temperature
// report has gone out so the link is known to be carrying traffic. The
// diagnostics cluster has no binding, so address the coordinator directly.
static void report_reset_count(void)
{
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {0};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    report_attr_cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    report_attr_cmd.zcl_basic_cmd.dst_endpoint = 1;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_TEMP_START_ENDPOINT;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_DIAGNOSTICS;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_DIAGNOSTICS_NUMBER_OF_RESETS_ID;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to report reset count: %s", esp_err_to_name(err));
    }
}

static void publish_remote_temp(void)
{
    if (!remote_temp_pending) {
        return;
    }
    remote_temp_pending = false;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_TEMP_START_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID, &zb_remote_temp_attr, false);
    esp_zb_lock_release();
    report_attr(HA_ESP_TEMP_START_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID);
}

static void diag_publish(void)
{
    diag_min_free_heap_kb = esp_get_minimum_free_heap_size() / 1024;
    if (zb_task_handle) {
        diag_stack_free_zb = uxTaskGetStackHighWaterMark(zb_task_handle);
    }
    TaskHandle_t timer_task = xTaskGetHandle("esp_timer");
    if (timer_task) {
        diag_stack_free_timer = uxTaskGetStackHighWaterMark(timer_task);
    }

    esp_zb_lock_acquire(portMAX_DELAY);
    for (uint16_t id = 0; id < DIAG_COUNT; id++) {
        esp_zb_zcl_set_attribute_val(HA_ESP_TEMP_START_ENDPOINT, ZB_DIAG_CLUSTER_ID,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, id, diag_values[id], false);
    }
    esp_zb_lock_release();
}

void esp_app_temp_sensor_handler(float temperature, uint8_t endpoint)
{
    // Update temperature in Thermostat cluster's local_temperature attribute
    int16_t measured_value = zb_temperature_to_s16(temperature);
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(endpoint,
                                 ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &measured_value, false);
    esp_zb_lock_release();
}


void send_on_off_command(uint8_t endpoint, bool on)
{
    esp_zb_zcl_on_off_cmd_t cmd_req;
    cmd_req.zcl_basic_cmd.src_endpoint = endpoint;
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    cmd_req.on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sent %s command from endpoint %d", on ? "ON" : "OFF", endpoint);
    } else {
        ESP_LOGW(TAG, "Failed to send %s command: %s", on ? "ON" : "OFF", esp_err_to_name(err));
    }
}

static void query_bound_switch_state(void)
{
    if (!startup_temp_received) {
        ESP_LOGD(TAG, "Skipping switch state query - no valid temp reading yet");
        return;
    }

    uint16_t attr_id = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
    esp_zb_zcl_read_attr_cmd_t cmd_req = {
        .zcl_basic_cmd.src_endpoint = HA_ESP_TEMP_START_ENDPOINT,
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .attr_number = 1,
        .attr_field = &attr_id,
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_read_attr_cmd_req(&cmd_req);
    esp_zb_lock_release();
    ESP_LOGI(TAG, "Querying bound switch state");
}

static esp_err_t zb_report_attr_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    CRUMB_ZB(CRUMB_ZB_REPORT_IN);
    diag_report_cb++;
    diag_report_cb_status = message->status;

    if (message->cluster != ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT ||
        message->attribute.id != ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
        ESP_LOGD(TAG, "Ignoring report: cluster=0x%x attr=0x%x", message->cluster, message->attribute.id);
        return ESP_OK;
    }

    if (message->status != ESP_ZB_ZCL_STATUS_SUCCESS ||
        message->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_S16 ||
        message->attribute.data.value == NULL) {
        ESP_LOGW(TAG, "Bad temperature report from 0x%04x ep %d: status=0x%x type=0x%x",
                 message->src_address.u.short_addr, message->src_endpoint,
                 message->status, message->attribute.data.type);
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw = *(int16_t *)message->attribute.data.value;
    if (raw == ZB_TEMP_MEASUREMENT_UNKNOWN) {
        ESP_LOGW(TAG, "Remote sensor 0x%04x reports unknown temperature", message->src_address.u.short_addr);
        return ESP_OK;
    }

    CRUMB_ZB(CRUMB_ZB_REPORT_OK);
    remote_temp = raw / 100.0f;
    remote_temp_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Remote temperature report from 0x%04x ep %d: %.2f°C (local %.2f°C)",
             message->src_address.u.short_addr, message->src_endpoint, remote_temp, last_fed_temp);

    zb_remote_temp_attr = raw;
    remote_temp_pending = true;
    CRUMB_ZB(CRUMB_ZB_REPORT_STORED);
    diag_report_stored++;
    CRUMB_ZB(CRUMB_ZB_REPORT_DONE);
    return ESP_OK;
}

// True while a received outdoor temperature that is not stale says it is too warm to need the heater.
// The gate closes above OUTDOOR_HEATER_LOCKOUT_ABOVE_C and reopens at or below OUTDOOR_HEATER_ALLOW_AT_C,
// keeping its state in between. A missing or stale reading opens it, so the heater never depends on a
// silent sensor. The report handler writes the reading on the Zigbee task and this runs on the timer task
// too, so a torn read of the 64-bit timestamp must fail safe: an impossible age counts as stale.
static bool outdoor_blocks_heater(void)
{
    static bool locked_out = false;
    bool was_locked_out = locked_out;

    int64_t received = remote_temp_time;
    int64_t age = esp_timer_get_time() - received;
    if (received == 0 || age < 0 || age > OUTDOOR_TEMP_STALE_US) {
        locked_out = false;
    } else if (remote_temp > OUTDOOR_HEATER_LOCKOUT_ABOVE_C) {
        locked_out = true;
    } else if (remote_temp <= OUTDOOR_HEATER_ALLOW_AT_C) {
        locked_out = false;
    }

    if (locked_out != was_locked_out) {
        ESP_LOGI(TAG, "Outdoor temperature %.2f°C: heater %s", remote_temp,
                 locked_out ? "kept off" : "allowed");
    }
    return locked_out;
}

static esp_err_t zb_read_attr_resp_handler(esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    CRUMB_ZB(CRUMB_ZB_READ_RESP);
    if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Read attr response status: 0x%x", message->info.status);
        return ESP_OK;
    }

    if (message->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        return ESP_OK;
    }

    esp_zb_zcl_read_attr_resp_variable_t *var = message->variables;
    while (var) {
        if (var->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
            var->status == ESP_ZB_ZCL_STATUS_SUCCESS) {

            bool actual_switch_state = *(bool *)var->attribute.data.value;
            ESP_LOGI(TAG, "Bound switch actual state: %s", actual_switch_state ? "ON" : "OFF");

            bool desired_state;
            if (outdoor_blocks_heater()) {
                desired_state = false;
            } else if (last_fed_temp == DSB1820_BAD_TEMP) {
                desired_state = true;
            } else if (last_fed_temp < temp_control_on_threshold) {
                desired_state = true;
            } else if (last_fed_temp >= temp_control_off_threshold) {
                desired_state = false;
            } else {
                desired_state = actual_switch_state;
            }

            if (actual_switch_state != desired_state) {
                ESP_LOGI(TAG, "Correcting switch state: %s -> %s (temp=%.2f)",
                         actual_switch_state ? "ON" : "OFF",
                         desired_state ? "ON" : "OFF",
                         last_fed_temp);
                send_on_off_command(HA_ESP_TEMP_START_ENDPOINT, desired_state);
            }
            switch_state = actual_switch_state;
        }
        var = var->next;
    }
    return ESP_OK;
}

void control_switch_by_temperature(float temperature, uint8_t endpoint)
{
    bool should_change_state = false;
    bool new_state = switch_state;

    // Set startup flag on first valid temperature reading
    if (temperature != DSB1820_BAD_TEMP && !startup_temp_received) {
        startup_temp_received = true;
        ESP_LOGI(TAG, "First valid temperature reading: %.2f°C - switch operations enabled", temperature);
    }

    // Block all switch operations until we have a valid temperature
    if (!startup_temp_received) {
        return;
    }

    // A fresh warm outdoor temperature outranks everything below, the failsafe included
    if (outdoor_blocks_heater()) {
        if (switch_state) {
            new_state = false;
            should_change_state = true;
            ESP_LOGI(TAG, "Outdoor temperature %.2f°C keeps the heater off - turning switch OFF", remote_temp);
        }
    }
    // Failsafe: If temperature reading is invalid, turn switch ON
    else if (temperature == DSB1820_BAD_TEMP) {
        if (!switch_state) {
            new_state = true;
            should_change_state = true;
            ESP_LOGW(TAG, "Temperature sensor failed - turning switch ON as failsafe");
        }
        // If already ON, keep it ON (don't log every time)
    }
    // Hysteresis logic to prevent rapid switching (only when temp is valid)
    else if (!switch_state && temperature < temp_control_on_threshold) {
        // Switch is OFF and temp dropped below ON threshold -> Turn ON
        new_state = true;
        should_change_state = true;
        ESP_LOGI(TAG, "Temperature %.2f°C dropped below ON threshold %.2f°C - turning switch ON",
                 temperature, temp_control_on_threshold);
    } else if (switch_state && temperature >= temp_control_off_threshold) {
        // Switch is ON and temp rose above OFF threshold -> Turn OFF
        new_state = false;
        should_change_state = true;
        ESP_LOGI(TAG, "Temperature %.2f°C rose above OFF threshold %.2f°C - turning switch OFF",
                 temperature, temp_control_off_threshold);
    } else {
        // In hysteresis zone - maintain current state
        ESP_LOGD(TAG, "Temperature %.2f°C in hysteresis zone (%.2f°C - %.2f°C) - switch remains %s",
                 temperature, temp_control_on_threshold, temp_control_off_threshold,
                 switch_state ? "ON" : "OFF");
    }

    if (should_change_state) {
        send_on_off_command(endpoint, new_state);
        switch_state = new_state;
    }
}

void read_temps(float *temp_results)
{
    float temperature;
    vTaskDelay(pdMS_TO_TICKS(200));
    for (uint8_t i = 0; i < ds18b20_device_num; i++)
    {
        esp_err_t err = ds18b20_trigger_temperature_conversion(ds18b20s[i]);
        if (err == ESP_OK)
        {
            err = ds18b20_get_temperature(ds18b20s[i], &temperature);
            if (err == ESP_OK)
            {
                temp_results[i] = temperature;
            }
            else
            {
                temp_results[i] = DSB1820_BAD_TEMP;
                read_failures++;
            }
        }
        else
        {
            temp_results[i] = DSB1820_BAD_TEMP;
            read_failures++;
        }
    }
    if (read_failures > (4 * HA_ESP_NUM_T_SENSORS))
    {
        // Had to be a way to reset the bus, but I haven't found it.
        onewire_bus_reset(ds18b20s[0]->bus);
    }
}
static void temp_timer_callback(void *arg)
{
    // ESP_LOGI(TAG, "Temperature timer called, time since boot: %lld us", time_since_boot);
    float temp_results[HA_ESP_NUM_T_SENSORS] = {DSB1820_BAD_TEMP};
    CRUMB_TIMER(CRUMB_TIMER_TEMP);
    read_temps(temp_results);
    CRUMB_TIMER(CRUMB_TIMER_READ);

    for (int i = 0; i < HA_ESP_NUM_T_SENSORS; i++)
    {
        uint8_t dsb_index = ep_to_ds[i];
        ESP_LOGD(TAG, "EP: %d, dsb_index: %d", i, dsb_index);

        if (dsb_index == 0xff)
        {
            ESP_LOGI(TAG, "No sensor for EP: %d", i);
        }
        else
        {
            ESP_LOGI(TAG, "temperature read from DS18B20[%d], for EP: %d,  %.2fC", dsb_index, i, temp_results[dsb_index]);

            // Feed watchdog with current reading
            feed_watchdog(temp_results[dsb_index]);

            // Control switch based on temperature (including failsafe for bad readings)
            CRUMB_TIMER(CRUMB_TIMER_CONTROL);
            control_switch_by_temperature(temp_results[dsb_index], (HA_ESP_TEMP_START_ENDPOINT + i));

            if (temp_results[dsb_index] != DSB1820_BAD_TEMP)
            {
                CRUMB_TIMER(CRUMB_TIMER_REPORT);
                esp_app_temp_sensor_handler(temp_results[dsb_index], (HA_ESP_TEMP_START_ENDPOINT + i));
                report_temp_attr(HA_ESP_TEMP_START_ENDPOINT + i);
                if (!reset_count_reported) {
                    reset_count_reported = true;
                    report_reset_count();
                }
            }
        }
    }

    CRUMB_TIMER(CRUMB_TIMER_REMOTE);
    publish_remote_temp();
    CRUMB_TIMER(CRUMB_TIMER_DIAG);
    diag_publish();
    CRUMB_TIMER(CRUMB_TIMER_DONE);
}


/* Steering retry backoff.
 *
 * The old code retried steering every second and dropped the retry chain
 * entirely if the stack rejected a start (ESP_RETURN_ON_FALSE). Both are wrong:
 * a fixed 1 s retry hammers the radio, and a dropped chain leaves the device
 * off the network until something else restarts it.
 */
static uint32_t s_steering_retry_delay_ms = 0;

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);

static void steering_retry_schedule(void) {
  if (s_steering_retry_delay_ms == 0) {
    s_steering_retry_delay_ms = 1000;
  } else {
    s_steering_retry_delay_ms *= 2;
    if (s_steering_retry_delay_ms > (5 * 60 * 1000)) {
      s_steering_retry_delay_ms = 5 * 60 * 1000;
    }
  }
  ESP_LOGW(TAG, "Scheduling another steering attempt in %lu ms",
           (unsigned long)s_steering_retry_delay_ms);
  esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                         ESP_ZB_BDB_MODE_NETWORK_STEERING,
                         s_steering_retry_delay_ms);
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to start BDB commissioning (mode 0x%x): %s",
             mode_mask, esp_err_to_name(err));
    if (mode_mask == ESP_ZB_BDB_MODE_NETWORK_STEERING &&
        !esp_zb_bdb_dev_joined()) {
      steering_retry_schedule();
    }
  }
}

static esp_err_t deferred_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing RGB LED");
    init_rgb_led();

    memset(ep_to_ds, 0xff, HA_ESP_NUM_T_SENSORS);

    bool onewire_bus_error = find_onewire(ds18b20s, &ds18b20_device_num);
    // We have an array of all found ds18b20s
    ESP_LOGI(TAG, "One Wire Count: %i", ds18b20_device_num);

    if (onewire_bus_error) {
        ESP_LOGE(TAG, "OneWire bus error - setting LED to RED");
        set_led_color(255, 0, 0);
    }

    uint8_t associated_eps_count = 0;
    // uint8_t unassociated_sensor_count = 0;
    //  For each found ds18b20, we need to try to match it to an endpoint
    for (uint8_t i = 0; i < HA_ESP_NUM_T_SENSORS; i++)
    {
        // for each endpoint see if we have a saved address
        onewire_device_address_t found_addr = 0;
        uint8_t *ep_index = &i;
        esp_err_t r_result = get_address_for_ep(&found_addr, ep_index);
        ESP_LOGI(TAG, "Err Result %d, address: %016llX", r_result, found_addr);
        if (r_result == ESP_OK && found_addr != 0)
        {
            // We have identified a saved address, but do we have a sensor that matches?
            bool dev_exists = 0x00;
            for (uint8_t j = 0; j < ds18b20_device_num; j++)
            {

                if (found_addr == ds18b20s[j]->addr)
                {
                    dev_exists = 0x01;
                    // associate dev with correct endpoint
                    ep_to_ds[j] = i;
                    ESP_LOGI(TAG, "DS18B20[%d], address: %016llX, associated with EP index: %d", j, found_addr, i);
                    associated_eps_count++;

                    break;
                }
            }

            if (!dev_exists)
            {
                // We no longer have it, so remove it from the saved state.
                ESP_LOGI(TAG, "Address: %016llX, associated with EP index: %d, deleted", found_addr, i);
                found_addr = 0;
                r_result = set_address_for_ep(&found_addr, ep_index);
            }
        }
    }

    // Ok, we have cleaned our list, now to assign extra sensors to endpoints
    // We know that anything with 0xff in ep_to_ds needs a sensor
    uint8_t available_sensors = ds18b20_device_num - associated_eps_count;
    uint8_t required_sensors = HA_ESP_NUM_T_SENSORS - associated_eps_count;

    ESP_LOGI(TAG, "There are %d sensors available to assign, need %d.", available_sensors, required_sensors);
    uint8_t av_sensor_index[available_sensors];
    memset(av_sensor_index, 0xff, available_sensors);

    uint8_t sensors_to_assign = available_sensors;
    if (available_sensors >= required_sensors)
    {
        sensors_to_assign = required_sensors;
    }

    for (uint8_t i = 0; i < sensors_to_assign; i++)
    {
        for (uint8_t j = 0; j < HA_ESP_NUM_T_SENSORS; j++)
        {
            if (ep_to_ds[j] == 0xff)
            {
                // Needs a sensor
                for (uint8_t k = 0; k < ds18b20_device_num; k++)
                {
                    bool sens_av = 0x01;
                    for (uint8_t l = 0; l < HA_ESP_NUM_T_SENSORS; l++)
                    {
                        if (ep_to_ds[l] == k)
                        {
                            sens_av = 0x00;
                            break;
                        }
                    }
                    if (sens_av)
                    {
                        ep_to_ds[j] = k;
                        uint64_t set_addr = (uint64_t)ds18b20s[k]->addr;
                        esp_err_t r_result = set_address_for_ep(&set_addr, &j);
                        if (r_result != ESP_OK)
                        {
                            ESP_LOGW(TAG, "Failed to save ep %d.", j);
                        }
                        ESP_LOGI(TAG, "Setting EP %d to Sensor index %d, address: %016llX", j, k, ds18b20s[k]->addr);
                        break;
                    }
                }
            }
        }
    }

    // Just print the final result so we can watch it over time.
    for (uint8_t i = 0; (i < HA_ESP_NUM_T_SENSORS) && (i < ds18b20_device_num); i++)
    {
        ESP_LOGI(TAG, "EP %d,  address: %016llX", i, ds18b20s[ep_to_ds[i]]->addr);
    }

    ESP_LOGI(TAG, "Starting Timers");
    start_temp_timer();
    return ESP_OK;
}

// Counts every ZCL frame before the stack dispatches it, so a report that arrives but never reaches
// zb_report_attr_handler() still shows up in the diagnostics. Returns false so normal processing continues.
static bool zb_raw_command_handler(uint8_t bufid)
{
    zb_zcl_parsed_hdr_t *hdr = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);

    CRUMB_ZB(CRUMB_ZB_RAW);
    diag_raw_total++;
    if (hdr->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT && hdr->is_common_command &&
        hdr->cmd_id == ZB_ZCL_CMD_REPORT_ATTRIBUTES) {
        diag_raw_reports++;
        diag_raw_report_src = hdr->addr_data.common_data.source.u.short_addr;
    }
    return false;
}

static esp_err_t esp_zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;

    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
        ret = zb_ota_upgrade_status_handler(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
        break;
    case ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID:
        ret = zb_ota_upgrade_query_image_resp_handler(*(esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
        ret = zb_identify_handler((esp_zb_zcl_identify_effect_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        // Default response from coordinator - normal Zigbee communication
        ESP_LOGD(TAG, "Received default response");
        break;
    case ESP_ZB_CORE_CMD_GREEN_POWER_RECV_CB_ID:
        // Green power cluster - not used, ignore silently
        break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_report_attr_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    default:
        CRUMB_ZB(CRUMB_ZB_OTHER_CB);
        diag_other_cb++;
        diag_other_cb_id = callback_id;
        ESP_LOGW(TAG, "Unhandled action callback: %d (0x%04x)", callback_id, callback_id);
        break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Action handler failed with error: %s", esp_err_to_name(ret));
    }

    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    CRUMB_ZB(CRUMB_ZB_SIGNAL);
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            s_steering_retry_delay_ms = 0;
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

            // Note: We don't report custom attributes here because the ESP Zigbee stack
            // doesn't properly support attribute reporting for custom clusters.
            // Initial values will need to be read by trying a write from zigbee2mqtt,
            // or the user can just set them via the frontend.
            ESP_LOGI(TAG, "Device ready - custom attributes: ON=%.2f°C, OFF=%.2f°C",
                     temp_control_on_threshold, temp_control_off_threshold);
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            steering_retry_schedule();
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE: {
        // A rejoin-type leave is handled by the stack itself. A reset-type leave
        // has already cleared the stack's state, so there is nothing to protect.
        esp_zb_zdo_signal_leave_params_t *leave_params =
            (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        uint8_t leave_type = leave_params ? leave_params->leave_type : ESP_ZB_NWK_LEAVE_TYPE_RESET;
        ESP_LOGW(TAG, "Left network (leave type: %d)", leave_type);
        if (leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            steering_retry_schedule();
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *custom_temperature_sensor_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Add Basic cluster for device identification
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,  // Mains (single phase) - this is a wall-powered router device. 0x03=battery, 0x04=dc
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Add Identify cluster for device identification (LED blink)
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Add Thermostat cluster for temperature threshold configuration
    // Initialize the global attribute values (in centidegrees)
    zb_on_threshold_attr = zb_temperature_to_s16(temp_control_on_threshold);
    zb_off_threshold_attr = zb_temperature_to_s16(temp_control_off_threshold);

    // Create thermostat cluster with our threshold values
    // We use: heating_setpoint = ON threshold, cooling_setpoint = OFF threshold
    esp_zb_thermostat_cluster_cfg_t thermostat_cfg = {
        .local_temperature = 0x8000,  // Invalid/unknown initially (will be updated by temp sensor)
        .occupied_cooling_setpoint = zb_off_threshold_attr,  // OFF threshold
        .occupied_heating_setpoint = zb_on_threshold_attr,   // ON threshold
        .control_sequence_of_operation = 0x04,  // Heating and Cooling
        .system_mode = 0x04,  // Heat mode
    };
    esp_zb_attribute_list_t *thermostat_cluster = esp_zb_thermostat_cluster_create(&thermostat_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(thermostat_cluster, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
                                            ESP_ZB_ZCL_ATTR_THERMOSTAT_OUTDOOR_TEMPERATURE_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_S16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                            &zb_remote_temp_attr));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_thermostat_cluster(cluster_list, thermostat_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Note: We use Thermostat cluster's local_temperature for our own temperature reporting.
    // The Temperature Measurement cluster is client-only: it receives reports from a bound
    // external sensor.
    esp_zb_temperature_meas_cluster_cfg_t temp_meas_cfg = {
        .measured_value = ZB_TEMP_MEASUREMENT_UNKNOWN,
        .min_value = ZB_TEMP_MEASUREMENT_UNKNOWN,
        .max_value = ZB_TEMP_MEASUREMENT_UNKNOWN,
    };
    esp_zb_attribute_list_t *temp_meas_cluster = esp_zb_temperature_meas_cluster_create(&temp_meas_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_meas_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    esp_zb_attribute_list_t *diag_attrs = esp_zb_zcl_attr_list_create(ZB_DIAG_CLUSTER_ID);
    for (uint16_t id = 0; id < DIAG_COUNT; id++) {
        ESP_ERROR_CHECK(esp_zb_cluster_add_attr(diag_attrs, ZB_DIAG_CLUSTER_ID, id, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                                ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                                diag_values[id]));
    }
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, diag_attrs, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Add Power Configuration cluster to report mains power
    esp_zb_power_config_cluster_cfg_t power_cfg = {
        .main_voltage = 0xffff, // Unknown mains voltage
        .main_alarm_mask = 0x00, // No alarms
    };
    esp_zb_attribute_list_t *power_cluster = esp_zb_power_config_cluster_create(&power_cfg);

    // Set battery percentage to 200 (0xC8) which indicates "AC/Mains powered" per Zigbee spec
    // This prevents battery low warnings in Zigbee2MQTT
    // Using raw attribute ID 0x0021 (BatteryPercentageRemaining)
    uint8_t battery_percentage = 200;
    ESP_ERROR_CHECK(esp_zb_power_config_cluster_add_attr(
        power_cluster, 0x0021,
        &battery_percentage));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(
        cluster_list, power_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Add On/Off client cluster to control other devices
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    // Add OTA upgrade client cluster
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_file_version = OTA_UPGRADE_FILE_VERSION,
        .ota_upgrade_downloaded_file_ver = OTA_UPGRADE_FILE_VERSION,
        .ota_upgrade_manufacturer = OTA_UPGRADE_MANUFACTURER,
        .ota_upgrade_image_type = OTA_UPGRADE_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);

    // Add additional required OTA attributes
    esp_zb_zcl_ota_upgrade_client_variable_t variable_config = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_UPGRADE_HW_VERSION,
        .max_data_size = OTA_UPGRADE_MAX_DATA_SIZE,
    };
    uint16_t ota_upgrade_server_addr = 0xffff;
    uint8_t ota_upgrade_server_ep = 0xff;

    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, (void *)&variable_config));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, (void *)&ota_upgrade_server_addr));
    ESP_ERROR_CHECK(esp_zb_ota_cluster_add_attr(ota_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, (void *)&ota_upgrade_server_ep));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(cluster_list, ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    // Add Diagnostics cluster for reset count
    esp_zb_attribute_list_t *diagnostics_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_DIAGNOSTICS);
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(diagnostics_cluster, ESP_ZB_ZCL_CLUSTER_ID_DIAGNOSTICS,
                                            ESP_ZB_ZCL_ATTR_DIAGNOSTICS_NUMBER_OF_RESETS_ID,
                                            ESP_ZB_ZCL_ATTR_TYPE_U16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
                                            &zb_reset_count));
    // Manufacturer-specific: forced re-joins
    ESP_ERROR_CHECK(esp_zb_cluster_add_attr(diagnostics_cluster, ESP_ZB_ZCL_CLUSTER_ID_DIAGNOSTICS,
                                            0xC001,
                                            ESP_ZB_ZCL_ATTR_TYPE_U16,
                                            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY,
                                            &zb_rejoin_count));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_diagnostics_cluster(cluster_list, diagnostics_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

static esp_zb_cluster_list_t *reboot_switch_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Add Basic cluster for device identification
    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,  // Mains powered
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Add Identify cluster
    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(&identify_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    // Add On/Off server cluster for reboot switch
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

static void reboot_switch_ep_create(esp_zb_ep_list_t *ep_list)
{
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_REBOOT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_cluster_list_t *cluster_list = reboot_switch_clusters_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
}

static void custom_temperature_sensor_ep_create(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id)
{
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_cluster_list_t *t_cl = custom_temperature_sensor_clusters_create();
    esp_zb_ep_list_add_ep(ep_list, t_cl, endpoint_config);
}

/* Remote/button rejoin (endpoint 11).
 *
 * A router has no parent to re-select, but leaving and re-steering rebuilds
 * its links and forces its children to pick a parent again - the repair for a
 * node wedged in a bad part of the mesh. It is disruptive (children are
 * orphaned until they rejoin), so it is rate limited and only ever explicit.
 */
static int64_t rejoin_blocked_until_us = 0;
static esp_timer_handle_t rejoin_timer = NULL;
static const char *pending_rejoin_reason = NULL;

static void rejoin_now(void)
{
    int64_t now = esp_timer_get_time();
    if (now < rejoin_blocked_until_us)
    {
        ESP_LOGW(TAG, "Rejoin ignored (%llds of rate limit left)",
                 (long long)((rejoin_blocked_until_us - now) / 1000000));
        return;
    }
    rejoin_blocked_until_us = now + (int64_t)REJOIN_MIN_INTERVAL_MS * 1000;
    zb_rejoin_count++;

    ESP_LOGW(TAG, "Rejoin requested (%s) - leaving the network to re-steer",
             pending_rejoin_reason ? pending_rejoin_reason : "unknown");

    /* Called from the esp_timer task, not the Zigbee task. */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_TEMP_START_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_DIAGNOSTICS,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 0xC001,
                                 &zb_rejoin_count, false);
    if (esp_zb_bdb_dev_joined())
    {
        /* Leave with reset: the LEAVE handler starts steering again. */
        esp_zb_bdb_reset_via_local_action();
    }
    else
    {
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
    esp_zb_lock_release();
}

static void rejoin_timer_callback(void *arg)
{
    rejoin_now();
}

/* Defer out of the ZCL callback: gives the stack time to acknowledge the
 * command, and runs the Zigbee calls on a task that can take the stack lock. */
static void rejoin_request_async(const char *reason)
{
    pending_rejoin_reason = reason;
    if (rejoin_timer == NULL)
    {
        const esp_timer_create_args_t args = {
            .callback = &rejoin_timer_callback,
            .name = "rejoin_timer",
        };
        if (esp_timer_create(&args, &rejoin_timer) != ESP_OK)
        {
            rejoin_now();
            return;
        }
    }
    esp_timer_stop(rejoin_timer);
    esp_timer_start_once(rejoin_timer, 200 * 1000);
}

static esp_zb_cluster_list_t *rejoin_switch_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,  // Mains powered
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,
    };
    esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}

static void rejoin_switch_ep_create(esp_zb_ep_list_t *ep_list)
{
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_REJOIN_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_cluster_list_t *cluster_list = rejoin_switch_clusters_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    for (uint8_t ep = HA_ESP_TEMP_START_ENDPOINT;
         ep < (HA_ESP_TEMP_START_ENDPOINT + HA_ESP_NUM_T_SENSORS);
         ep++)
    {
        custom_temperature_sensor_ep_create(ep_list, ep);
    }

    // Add reboot switch endpoint
    reboot_switch_ep_create(ep_list);
    rejoin_switch_ep_create(ep_list);

    // Register OTA upgrade action handler
    esp_zb_core_action_handler_register(esp_zb_action_handler);
    esp_zb_raw_command_handler_register(zb_raw_command_handler);

    esp_zb_device_register(ep_list);
    for (uint8_t ep = HA_ESP_TEMP_START_ENDPOINT;
         ep < (HA_ESP_TEMP_START_ENDPOINT + HA_ESP_NUM_T_SENSORS);
         ep++)
    {
        // Configure reporting for Thermostat cluster's local_temperature
        esp_zb_zcl_reporting_info_t reporting_info = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = ep,
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .dst.endpoint = 1,
            .u.send_info.min_interval = 1,
            .u.send_info.max_interval = 300,
            .u.send_info.def_min_interval = 1,
            .u.send_info.def_max_interval = 300,
            .u.send_info.delta.u16 = 50,  // 0.5°C change
            .attr_id = ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };

        ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_info));

        // Zigbee2MQTT configures battery reporting, and a ConfigureReporting for
        // an attribute with no record here creates one with manufacturer code 0.
        // The stack cannot resolve that record and dereferences NULL in its
        // reporting timer, so register it first and let the command update it.
        esp_zb_zcl_reporting_info_t power_reporting_info = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = ep,
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .dst.endpoint = 1,
            .u.send_info.min_interval = 3600,
            .u.send_info.max_interval = 65000,
            .u.send_info.def_min_interval = 3600,
            .u.send_info.def_max_interval = 65000,
            .u.send_info.delta.u8 = 10,
            .attr_id = 0x0021,  // BatteryPercentageRemaining
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };
        esp_err_t power_err = esp_zb_zcl_update_reporting_info(&power_reporting_info);
        if (power_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register battery reporting: %s", esp_err_to_name(power_err));
        }
    }

    // The same applies to the reboot switch, which Zigbee2MQTT binds on genOnOff
    // and reads.
    esp_zb_zcl_reporting_info_t reboot_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_REBOOT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .dst.endpoint = 1,
        .u.send_info.min_interval = 0,
        .u.send_info.max_interval = 65000,
        .u.send_info.def_min_interval = 0,
        .u.send_info.def_max_interval = 65000,
        .attr_id = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_err_t reboot_err = esp_zb_zcl_update_reporting_info(&reboot_reporting_info);
    if (reboot_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register reboot switch reporting: %s", esp_err_to_name(reboot_err));
    }

    // The one-shot reset count report needs a reporting record for its
    // attribute. Periodic reporting stays off (max interval 0xFFFF).
    esp_zb_zcl_reporting_info_t diagnostics_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_TEMP_START_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_DIAGNOSTICS,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .dst.endpoint = 1,
        .u.send_info.min_interval = 0,
        .u.send_info.max_interval = 0xFFFF,
        .u.send_info.def_min_interval = 0,
        .u.send_info.def_max_interval = 0xFFFF,
        .u.send_info.delta.u16 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_DIAGNOSTICS_NUMBER_OF_RESETS_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_err_t diag_err = esp_zb_zcl_update_reporting_info(&diagnostics_reporting_info);
    if (diag_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register reset count reporting: %s", esp_err_to_name(diag_err));
    }

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    diag_reset_reason = reason;
    if (rtc_crumb_magic == CRUMB_MAGIC && reason != ESP_RST_POWERON) {
        diag_crumb_zb_prev = rtc_crumb_zb;
        diag_crumb_timer_prev = rtc_crumb_timer;
        diag_restart_cause_prev = rtc_restart_cause;
    }
    rtc_crumb_magic = CRUMB_MAGIC;
    rtc_crumb_zb = 0;
    rtc_crumb_timer = 0;
    rtc_restart_cause = 0;
    ESP_ERROR_CHECK(esp_register_shutdown_handler(restart_shutdown_handler));
    ESP_LOGW(TAG, "Boot, reset reason %d, previous breadcrumbs zb=%u timer=%u restart cause=%u",
             reason, diag_crumb_zb_prev, diag_crumb_timer_prev, diag_restart_cause_prev);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Increment and store reset count before Zigbee starts
    zb_reset_count = increment_and_get_reset_count();

    // Load the saved thresholds before the thermostat cluster is created, so the setpoint attributes
    // Zigbee2MQTT reads start from the values actually in use
    float loaded_on = 0, loaded_off = 0;
    if (get_temp_thresholds(&loaded_on, &loaded_off) == ESP_OK) {
        temp_control_on_threshold = loaded_on;
        temp_control_off_threshold = loaded_off;
        ESP_LOGI(TAG, "Using stored thresholds: ON=%.2f°C, OFF=%.2f°C",
                 temp_control_on_threshold, temp_control_off_threshold);
    } else {
        ESP_LOGI(TAG, "Using default thresholds: ON=%.2f°C, OFF=%.2f°C",
                 temp_control_on_threshold, temp_control_off_threshold);
    }

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", ZB_TASK_STACK_SIZE, NULL, 5, &zb_task_handle);
}
