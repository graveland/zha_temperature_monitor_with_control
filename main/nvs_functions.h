#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "onewire_bus.h"
#include "esp_log.h"

static const char *NTAG = "ESP_ZB_NVS";

const char *ep_to_key[4] = {"ep0", "ep1", "ep2", "ep3"};

void init_flash()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t get_address_for_ep(onewire_device_address_t *ep_addr, uint8_t *end_point)
{
    ESP_LOGI(NTAG, "Ep %d", *end_point);
    const char *ep_key = ep_to_key[*end_point];
    ESP_LOGI(NTAG, "Getting %s", ep_key);
    init_flash();

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("onewire_storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(NTAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        uint64_t out = 0;

        err = nvs_get_u64(nvs_handle, ep_key, &out);
        if (err == ESP_OK)
        {
            ESP_LOGI(NTAG, "%s:address: %016llX ", ep_key, out);
            memcpy(ep_addr, &out, 8);

            // ep_addr = (onewire_device_address_t)out;
        }
        else
        {
            ESP_LOGE(NTAG, "Error (%s) reading!", esp_err_to_name(err));
        }
    }
    nvs_close(nvs_handle);
    return err;
}

esp_err_t set_address_for_ep(onewire_device_address_t *ep_addr, uint8_t *end_point)
{
    ESP_LOGI(NTAG, "Ep %d", *end_point);

    const char *ep_key = ep_to_key[*end_point];
    ESP_LOGI(NTAG, "Setting %s", ep_key);
    init_flash();

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("onewire_storage", NVS_READWRITE, &nvs_handle);
    ESP_LOGI(NTAG, "Opened Store");
    if (err != ESP_OK)
    {
        ESP_LOGE(NTAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        uint64_t out = 0;
        //(uintptr_t)ep_addr;
        memcpy(&out, ep_addr, 8);

        err = nvs_set_u64(nvs_handle, ep_key, out);
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    ESP_LOGI(NTAG, "Closed Store");

    return err;
}

esp_err_t get_temp_thresholds(float *on_threshold, float *off_threshold)
{
    init_flash();

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("temp_config", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGI(NTAG, "No stored temperature thresholds, will use defaults");
        return err;
    }

    int32_t on_value = 0;
    int32_t off_value = 0;

    esp_err_t on_err = nvs_get_i32(nvs_handle, "on_thresh", &on_value);
    esp_err_t off_err = nvs_get_i32(nvs_handle, "off_thresh", &off_value);

    if (on_err == ESP_OK && off_err == ESP_OK)
    {
        *on_threshold = (float)on_value / 100.0f;
        *off_threshold = (float)off_value / 100.0f;
        ESP_LOGI(NTAG, "Loaded thresholds: ON=%.2f°C, OFF=%.2f°C", *on_threshold, *off_threshold);
    }
    else
    {
        ESP_LOGI(NTAG, "Error reading thresholds: on=%s, off=%s",
                 esp_err_to_name(on_err), esp_err_to_name(off_err));
        err = ESP_FAIL;
    }

    nvs_close(nvs_handle);
    return err;
}

esp_err_t set_temp_thresholds(float on_threshold, float off_threshold)
{
    init_flash();

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("temp_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(NTAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    // Store as int32 (multiply by 100 to preserve 2 decimal places)
    int32_t on_value = (int32_t)(on_threshold * 100.0f);
    int32_t off_value = (int32_t)(off_threshold * 100.0f);

    err = nvs_set_i32(nvs_handle, "on_thresh", on_value);
    if (err == ESP_OK)
    {
        err = nvs_set_i32(nvs_handle, "off_thresh", off_value);
    }

    if (err == ESP_OK)
    {
        err = nvs_commit(nvs_handle);
        ESP_LOGI(NTAG, "Saved thresholds: ON=%.2f°C, OFF=%.2f°C", on_threshold, off_threshold);
    }
    else
    {
        ESP_LOGE(NTAG, "Error saving thresholds: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}
