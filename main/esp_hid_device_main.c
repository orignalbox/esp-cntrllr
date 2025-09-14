/*
 * ESP32-S3 BLE Standard Gamepad
 *
 * Maps 4 GPIOs to a Joystick X/Y Axis and 4 GPIOs to Action Buttons.
 * This is the standard layout most games (e.g., Minecraft, Simulators) expect.
 *
 * RGB LED States:
 * 1. Advertising: Pulsing dim Blue
 * 2. Connected (Idle): Solid dim White
 * 3. Button Press: Bright, unique color per button.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h> // Needed for HSV pulse

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

// DRIVERS
#include "driver/gpio.h"
#include "led_strip.h" // For RGB LED

// NIMBLE (BLE) STACK
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

// ESP-HID DRIVERS
#include "esp_hidd.h"
#include "esp_hid_gap.h" // Includes helper functions

static const char *TAG = "GAMEPAD_DEMO";

// --- New Logical Button Definitions ---
// D-Pad (Mapped to Joystick Axes)
#define BUTTON_UP_GPIO      GPIO_NUM_18
#define BUTTON_DOWN_GPIO    GPIO_NUM_16
#define BUTTON_LEFT_GPIO    GPIO_NUM_15
#define BUTTON_RIGHT_GPIO   GPIO_NUM_8

// Action Buttons
#define BUTTON_A_GPIO       GPIO_NUM_1  // Action 1 (Cross/A)
#define BUTTON_B_GPIO       GPIO_NUM_2  // Action 2 (Circle/B)
#define BUTTON_C_GPIO       GPIO_NUM_4  // Action 3 (Square/X)
#define BUTTON_D_GPIO       GPIO_NUM_17 // Action 4 (Triangle/Y)

// Bitmask for all 8 buttons
#define BUTTON_PIN_SEL    ((1ULL<<BUTTON_UP_GPIO)   | (1ULL<<BUTTON_DOWN_GPIO) | \
                           (1ULL<<BUTTON_LEFT_GPIO) | (1ULL<<BUTTON_RIGHT_GPIO) | \
                           (1ULL<<BUTTON_A_GPIO) | (1ULL<<BUTTON_B_GPIO) | \
                           (1ULL<<BUTTON_C_GPIO) | (1ULL<<BUTTON_D_GPIO))

// --- LED Definition ---
#define LED_STRIP_GPIO    GPIO_NUM_38
static led_strip_handle_t led_strip;

// --- Global Connection State ---
static volatile bool s_ble_connected = false;

typedef struct
{
    TaskHandle_t task_hdl; // Task handle for the gamepad task
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_ble_hid_param = {0};

// --- NEW HID Report Map (Joystick + 4 Buttons) ---
// This creates a 3-byte report:
// Byte 0: X-Axis (-127 to 127)
// Byte 1: Y-Axis (-127 to 127)
// Byte 2: 4 Buttons (as 4 bits) + 4 bits padding
const unsigned char gamepadReportMap[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Gamepad)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    
    // X and Y Axes (2 bytes)
    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x15, 0x81,       //   Logical Minimum (-127)
    0x25, 0x7F,       //   Logical Maximum (127)
    0x75, 0x08,       //   Report Size (8 bits)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x02,       //   Input (Data, Var, Abs)
    
    // 4 Action Buttons (1 byte, 4 bits used, 4 bits padding)
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (Button 1)
    0x29, 0x04,       //   Usage Maximum (Button 4)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1 bit)
    0x95, 0x04,       //   Report Count (4)
    0x81, 0x02,       //   Input (Data, Var, Abs)

    // 4 bits of padding
    0x75, 0x04,       //   Report Size (4 bits)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x03,       //   Input (Const, Var, Abs)
    
    0xC0              // End Collection
};

// --- NEW Report Struct ---
// This must match the report map: 3 bytes
typedef struct {
    int8_t  x_axis;     // Byte 0: X-Axis
    int8_t  y_axis;     // Byte 1: Y-Axis
    uint8_t buttons;    // Byte 2: Buttons (bits 0-3)
} gamepad_report_t;

// Gamepad report state
static gamepad_report_t s_gamepad_report;
static gamepad_report_t s_last_gamepad_report;


static esp_hid_raw_report_map_t ble_report_maps[] = {
    {
        .data = gamepadReportMap,
        .len = sizeof(gamepadReportMap)
    }
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id          = 0x16C0,
    .product_id         = 0x05DF,
    .version            = 0x0100,
    .device_name        = "ESP Gamepad",
    .manufacturer_name  = "Espressif",
    .serial_number      = "1234567890",
    .report_maps        = ble_report_maps,
    .report_maps_len    = 1
};

void hsv_to_rgb(uint16_t h, double s, double v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    double c = v * s;
    double h_prime = fmod(h / 60.0, 6);
    double x = c * (1 - fabs(fmod(h_prime, 2) - 1));
    double m = v - c;
    double r_temp = 0, g_temp = 0, b_temp = 0;

    if (0 <= h_prime && h_prime < 1) {
        r_temp = c; g_temp = x; b_temp = 0;
    } else if (1 <= h_prime && h_prime < 2) {
        r_temp = x; g_temp = c; b_temp = 0;
    } else if (2 <= h_prime && h_prime < 3) {
        r_temp = 0; g_temp = c; b_temp = x;
    } else if (3 <= h_prime && h_prime < 4) {
        r_temp = 0; g_temp = x; b_temp = c;
    } else if (4 <= h_prime && h_prime < 5) {
        r_temp = x; g_temp = 0; b_temp = c;
    } else if (5 <= h_prime && h_prime < 6) {
        r_temp = c; g_temp = 0; b_temp = x;
    }
    *r = (uint8_t)((r_temp + m) * 255);
    *g = (uint8_t)((g_temp + m) * 255);
    *b = (uint8_t)((b_temp + m) * 255);
}

// Send the 3-byte gamepad report
void send_gamepad_report(gamepad_report_t *report)
{
    if (s_ble_connected) {
        esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 1, (uint8_t*)report, sizeof(gamepad_report_t));
    }
}

// --- Gamepad Polling Task (Now also handles LED) ---
void gamepad_poll_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Gamepad polling & LED task started");
    static uint16_t s_pulse_hue = 240; // Blue for advertising pulse
    static float s_pulse_val = 0.0;
    static bool s_pulse_dir_up = true;
    uint8_t r, g, b;

    // Clear report states on start
    memset(&s_gamepad_report, 0, sizeof(gamepad_report_t));
    memset(&s_last_gamepad_report, 0, sizeof(gamepad_report_t));

    while (1) {
        // --- 1. Reset current report ---
        memset(&s_gamepad_report, 0, sizeof(gamepad_report_t));

        // --- 2. Read Axes ---
        if (gpio_get_level(BUTTON_UP_GPIO) == 0) {
            s_gamepad_report.y_axis = -127; // Y-Axis Up
        } else if (gpio_get_level(BUTTON_DOWN_GPIO) == 0) {
            s_gamepad_report.y_axis = 127;  // Y-Axis Down
        }

        if (gpio_get_level(BUTTON_LEFT_GPIO) == 0) {
            s_gamepad_report.x_axis = -127; // X-Axis Left
        } else if (gpio_get_level(BUTTON_RIGHT_GPIO) == 0) {
            s_gamepad_report.x_axis = 127;  // X-Axis Right
        }

        // --- 3. Read 4 Action Buttons ---
        if (gpio_get_level(BUTTON_A_GPIO) == 0) {
            s_gamepad_report.buttons |= (1 << 0); // Button 1
        }
        if (gpio_get_level(BUTTON_B_GPIO) == 0) {
            s_gamepad_report.buttons |= (1 << 1); // Button 2
        }
        if (gpio_get_level(BUTTON_C_GPIO) == 0) {
            s_gamepad_report.buttons |= (1 << 2); // Button 3
        }
        if (gpio_get_level(BUTTON_D_GPIO) == 0) {
            s_gamepad_report.buttons |= (1 << 3); // Button 4
        }

        // --- 4. Send BLE report if state changed (only if connected) ---
        if (s_ble_connected && (memcmp(&s_gamepad_report, &s_last_gamepad_report, sizeof(gamepad_report_t)) != 0)) {
            ESP_LOGI(TAG, "Report changed: X=%d, Y=%d, BTN=%d", s_gamepad_report.x_axis, s_gamepad_report.y_axis, s_gamepad_report.buttons);
            send_gamepad_report(&s_gamepad_report);
            memcpy(&s_last_gamepad_report, &s_gamepad_report, sizeof(gamepad_report_t));
        }

        // --- 5. Update RGB LED based on state ---
        if (s_ble_connected) {
            bool dpad_pressed = (s_gamepad_report.x_axis != 0 || s_gamepad_report.y_axis != 0);
            bool button_pressed = (s_gamepad_report.buttons != 0);

            if (!dpad_pressed && !button_pressed) {
                // STATE: Connected & Idle -> Solid dim white
                led_strip_set_pixel(led_strip, 0, 10, 10, 10);
            } else {
                // STATE: Button Pressed -> Bright color
                if (dpad_pressed) { 
                    led_strip_set_pixel(led_strip, 0, 80, 80, 80); // Bright White
                } else if (s_gamepad_report.buttons & (1 << 0)) {  // Button A (Cross)
                    led_strip_set_pixel(led_strip, 0, 0, 0, 80);   // Bright Blue
                } else if (s_gamepad_report.buttons & (1 << 1)) {  // Button B (Circle)
                    led_strip_set_pixel(led_strip, 0, 80, 0, 0);   // Bright Red
                } else if (s_gamepad_report.buttons & (1 << 2)) {  // Button C (Square)
                    led_strip_set_pixel(led_strip, 0, 80, 0, 80);   // Bright Pink
                } else if (s_gamepad_report.buttons & (1 << 3)) {  // Button D (Triangle)
                    led_strip_set_pixel(led_strip, 0, 0, 80, 0);   // Bright Green
                }
            }
        } else {
            // STATE: Advertising (Not Connected) -> Pulsing dim blue
            hsv_to_rgb(s_pulse_hue, 1.0, s_pulse_val, &r, &g, &b);
            led_strip_set_pixel(led_strip, 0, r, g, b);

            // Update pulse brightness
            if (s_pulse_dir_up) {
                s_pulse_val += 0.005; // Fade in speed
                if (s_pulse_val >= 0.1) { // Max brightness (dim)
                    s_pulse_val = 0.1;
                    s_pulse_dir_up = false;
                }
            } else {
                s_pulse_val -= 0.005; // Fade out speed
                if (s_pulse_val <= 0.0) {
                    s_pulse_val = 0.0;
                    s_pulse_dir_up = true;
                }
            }
        }
        
        // Always refresh the LED
        led_strip_refresh(led_strip);
        
        // Wait 20ms before polling again
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG_BLE = "HID_DEV_BLE";

    switch (event) {
    case ESP_HIDD_START_EVENT: {
        ESP_LOGI(TAG_BLE, "START");
        esp_hid_ble_gap_adv_start();
        s_ble_connected = false; // Set connected flag to false
        break;
    }
    case ESP_HIDD_CONNECT_EVENT: {
        ESP_LOGI(TAG_BLE, "CONNECT");
        s_ble_connected = true; // Set connected flag to true
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT: {
        ESP_LOGI(TAG_BLE, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT: {
        ESP_LOGI(TAG_BLE, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT: {
        ESP_LOGI(TAG_BLE, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG_BLE, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT: {
        ESP_LOGI(TAG_BLE, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG_BLE, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT: {
        ESP_LOGI(TAG_BLE, "DISCONNECT: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
        s_ble_connected = false; // Set connected flag to false
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_STOP_EVENT: {
        ESP_LOGI(TAG_BLE, "STOP");
        break;
    }
    // Added to fix -Werror=switch
    case ESP_HIDD_ANY_EVENT:
        break;
    case ESP_HIDD_MAX_EVENT:
        break;
    default:
        break;
    } 
    return;
}

// --- LED Configuration Function (with mem_block fix) ---
static void configure_led(void)
{
    ESP_LOGI(TAG, "Configuring RGB LED on GPIO %d", LED_STRIP_GPIO);
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .mem_block_symbols = 64, // The critical fix
        .flags.with_dma = false,
    };
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip); // Turn off LED on startup
    ESP_LOGI(TAG, "RGB LED configured");
}

// This dummy function fixes the linker error from esp_hid_gap.c
void ble_hid_task_start_up(void)
{
    // Do nothing. Task is started from app_main.
}

void ble_hid_device_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Implemented for NimBLE compatibility
void ble_store_config_init(void); 

void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK( ret );

    // --- Configure Buttons ---
    ESP_LOGI(TAG, "Configuring GPIOs for buttons");
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = BUTTON_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "GPIO configuration complete");

    // --- Configure LED ---
    configure_led();

    // --- Configure BLE ---
    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GAMEPAD, ble_hid_config.device_name);
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));

    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    
    ret = esp_nimble_enable(ble_hid_device_host_task);
    if (ret) {
        ESP_LOGE(TAG, "esp_nimble_enable failed: %d", ret);
    }

    // --- Start the main polling task ---
    // This task now runs continuously to manage buttons and LED state
    xTaskCreate(gamepad_poll_task, "gamepad_poll_task", 3 * 1024, NULL, configMAX_PRIORITIES - 3,
                &s_ble_hid_param.task_hdl);
}