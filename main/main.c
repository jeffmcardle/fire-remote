#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"

// CMakeLists.txt for main/ should include:
// idf_component_register(SRCS "main.c"
//                       INCLUDE_DIRS "."
//                       REQUIRES nvs_flash bt driver)

#define HID_TAG "BLE_HID"
#define HID_DEVICE_NAME "Fire TV Remote"

// GPIO definitions for ESP32-C3 Super Mini
#define BTN_UP      GPIO_NUM_2
#define BTN_DOWN    GPIO_NUM_3
#define BTN_LEFT    GPIO_NUM_4
#define BTN_RIGHT   GPIO_NUM_5
#define BTN_SELECT  GPIO_NUM_6
#define BTN_BACK    GPIO_NUM_7
#define BTN_HOME    GPIO_NUM_8
#define BTN_PLAY    GPIO_NUM_9
#define BTN_VOL_UP  GPIO_NUM_10
#define BTN_VOL_DN  GPIO_NUM_20

// HID Consumer Control Usage IDs
#define HID_CC_PLAY_PAUSE   0xCD
#define HID_CC_VOLUME_UP    0xE9
#define HID_CC_VOLUME_DOWN  0xEA
#define HID_CC_HOME         0x223

// GATT Service handles
#define GATTS_SERVICE_UUID_HID   0x1812
#define GATTS_CHAR_UUID_HID_REPORT_MAP   0x2A4B
#define GATTS_CHAR_UUID_HID_REPORT       0x2A4D
#define GATTS_CHAR_UUID_HID_INFO         0x2A4A

#define GATTS_NUM_HANDLE_HID    8
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 10

static uint8_t hid_service_uuid[16] = {
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

// HID Report Descriptor
static uint8_t hid_report_map[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x19, 0x00,        //   Usage Minimum (0)
    0x2A, 0x3C, 0x02,  //   Usage Maximum (0x23C)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0x3C, 0x02,  //   Logical Maximum (0x23C)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x10,        //   Report Size (16)
    0x81, 0x00,        //   Input (Data,Array,Abs)
    0xC0,              // End Collection
    
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x01,        //   Input (Const)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0xFF,        //   Usage Maximum (255)
    0x81, 0x00,        //   Input (Data,Array)
    0xC0               // End Collection
};

static uint8_t hid_info_char_val[] = {0x11, 0x01, 0x00, 0x03};

static uint16_t hid_handle_table[GATTS_NUM_HANDLE_HID];

static uint16_t hid_conn_id = 0;
static esp_gatt_if_t hid_gatts_if = 0;
static bool is_connected = false;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_data[] = {
    0x02, 0x01, 0x06,
    0x03, 0x03, 0x12, 0x18,
    0x0E, 0x09, 'F', 'i', 'r', 'e', ' ', 'T', 'V', ' ', 'R', 'e', 'm', 'o', 't', 'e'
};

void send_consumer_control(uint16_t usage_id) {
    if (!is_connected || hid_gatts_if == 0) return;
    
    uint8_t report[3];
    report[0] = 0x01;  // Report ID
    report[1] = usage_id & 0xFF;
    report[2] = (usage_id >> 8) & 0xFF;
    
    esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id, 
                                hid_handle_table[3], sizeof(report), report, false);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    report[1] = 0x00;
    report[2] = 0x00;
    esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id, 
                                hid_handle_table[3], sizeof(report), report, false);
}

void send_keyboard_key(uint8_t keycode) {
    if (!is_connected || hid_gatts_if == 0) return;
    
    uint8_t report[9] = {0x02, 0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00};
    esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id, 
                                hid_handle_table[3], sizeof(report), report, false);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    memset(report, 0, sizeof(report));
    report[0] = 0x02;
    esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id, 
                                hid_handle_table[3], sizeof(report), report, false);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        default:
            break;
    }
}

static const esp_gatts_attr_db_t hid_gatt_db[GATTS_NUM_HANDLE_HID] = {
    // Service Declaration
    [0] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ESP_GATT_UUID_PRI_SERVICE, 
           ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_HID), 
           (uint8_t *)&GATTS_SERVICE_UUID_HID}},

    // HID Information Characteristic Declaration
    [1] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ESP_GATT_UUID_CHAR_DECLARE, 
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), 
           (uint8_t *)&ESP_GATT_CHAR_PROP_BIT_READ}},

    // HID Information Characteristic Value
    [2] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HID_INFO, 
           ESP_GATT_PERM_READ, sizeof(hid_info_char_val), sizeof(hid_info_char_val), 
           hid_info_char_val}},

    // HID Report Map Characteristic Declaration
    [3] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ESP_GATT_UUID_CHAR_DECLARE, 
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), 
           (uint8_t *)&ESP_GATT_CHAR_PROP_BIT_READ}},

    // HID Report Map Characteristic Value
    [4] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HID_REPORT_MAP, 
           ESP_GATT_PERM_READ, sizeof(hid_report_map), sizeof(hid_report_map), 
           hid_report_map}},

    // HID Report Characteristic Declaration
    [5] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ESP_GATT_UUID_CHAR_DECLARE, 
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t), 
           (uint8_t *)&ESP_GATT_CHAR_PROP_BIT_READ_NOTIFY}},

    // HID Report Characteristic Value
    [6] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HID_REPORT, 
           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}},

    // HID Report Client Characteristic Configuration Descriptor
    [7] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ESP_GATT_UUID_CHAR_CLIENT_CONFIG, 
           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), 0, NULL}},
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(HID_TAG, "GATT server registered, app_id %04x", param->reg.app_id);
            
            esp_ble_gap_set_device_name(HID_DEVICE_NAME);
            esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
            
            esp_ble_gatts_create_attr_tab(hid_gatt_db, gatts_if, GATTS_NUM_HANDLE_HID, 0);
            break;
            
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(HID_TAG, "Create attribute table failed, error code=0x%x", 
                         param->add_attr_tab.status);
            } else if (param->add_attr_tab.num_handle != GATTS_NUM_HANDLE_HID) {
                ESP_LOGE(HID_TAG, "Create attribute table abnormally");
            } else {
                ESP_LOGI(HID_TAG, "Attribute table created successfully");
                memcpy(hid_handle_table, param->add_attr_tab.handles, sizeof(hid_handle_table));
                esp_ble_gatts_start_service(hid_handle_table[0]);
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(HID_TAG, "Device connected, conn_id %d", param->connect.conn_id);
            hid_conn_id = param->connect.conn_id;
            hid_gatts_if = gatts_if;
            is_connected = true;
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(HID_TAG, "Device disconnected");
            is_connected = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        default:
            break;
    }
}

void button_task(void *pvParameters) {
    uint8_t last_state[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    
    while (1) {
        if (gpio_get_level(BTN_UP) == 0 && last_state[0] == 1) {
            ESP_LOGI(HID_TAG, "UP pressed");
            send_keyboard_key(0x52);
            last_state[0] = 0;
        } else if (gpio_get_level(BTN_UP) == 1) {
            last_state[0] = 1;
        }
        
        if (gpio_get_level(BTN_DOWN) == 0 && last_state[1] == 1) {
            ESP_LOGI(HID_TAG, "DOWN pressed");
            send_keyboard_key(0x51);
            last_state[1] = 0;
        } else if (gpio_get_level(BTN_DOWN) == 1) {
            last_state[1] = 1;
        }
        
        if (gpio_get_level(BTN_LEFT) == 0 && last_state[2] == 1) {
            ESP_LOGI(HID_TAG, "LEFT pressed");
            send_keyboard_key(0x50);
            last_state[2] = 0;
        } else if (gpio_get_level(BTN_LEFT) == 1) {
            last_state[2] = 1;
        }
        
        if (gpio_get_level(BTN_RIGHT) == 0 && last_state[3] == 1) {
            ESP_LOGI(HID_TAG, "RIGHT pressed");
            send_keyboard_key(0x4F);
            last_state[3] = 0;
        } else if (gpio_get_level(BTN_RIGHT) == 1) {
            last_state[3] = 1;
        }
        
        if (gpio_get_level(BTN_SELECT) == 0 && last_state[4] == 1) {
            ESP_LOGI(HID_TAG, "SELECT pressed");
            send_keyboard_key(0x28);
            last_state[4] = 0;
        } else if (gpio_get_level(BTN_SELECT) == 1) {
            last_state[4] = 1;
        }
        
        if (gpio_get_level(BTN_BACK) == 0 && last_state[5] == 1) {
            ESP_LOGI(HID_TAG, "BACK pressed");
            send_keyboard_key(0x29);
            last_state[5] = 0;
        } else if (gpio_get_level(BTN_BACK) == 1) {
            last_state[5] = 1;
        }
        
        if (gpio_get_level(BTN_HOME) == 0 && last_state[6] == 1) {
            ESP_LOGI(HID_TAG, "HOME pressed");
            send_consumer_control(HID_CC_HOME);
            last_state[6] = 0;
        } else if (gpio_get_level(BTN_HOME) == 1) {
            last_state[6] = 1;
        }
        
        if (gpio_get_level(BTN_PLAY) == 0 && last_state[7] == 1) {
            ESP_LOGI(HID_TAG, "PLAY/PAUSE pressed");
            send_consumer_control(HID_CC_PLAY_PAUSE);
            last_state[7] = 0;
        } else if (gpio_get_level(BTN_PLAY) == 1) {
            last_state[7] = 1;
        }
        
        if (gpio_get_level(BTN_VOL_UP) == 0 && last_state[8] == 1) {
            ESP_LOGI(HID_TAG, "VOL UP pressed");
            send_consumer_control(HID_CC_VOLUME_UP);
            last_state[8] = 0;
        } else if (gpio_get_level(BTN_VOL_UP) == 1) {
            last_state[8] = 1;
        }
        
        if (gpio_get_level(BTN_VOL_DN) == 0 && last_state[9] == 1) {
            ESP_LOGI(HID_TAG, "VOL DOWN pressed");
            send_consumer_control(HID_CC_VOLUME_DOWN);
            last_state[9] = 0;
        } else if (gpio_get_level(BTN_VOL_DN) == 1) {
            last_state[9] = 1;
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void init_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_UP) | (1ULL << BTN_DOWN) | (1ULL << BTN_LEFT) | 
                        (1ULL << BTN_RIGHT) | (1ULL << BTN_SELECT) | (1ULL << BTN_BACK) |
                        (1ULL << BTN_HOME) | (1ULL << BTN_PLAY) | (1ULL << BTN_VOL_UP) | 
                        (1ULL << BTN_VOL_DN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

void app_main(void) {
    esp_err_t ret;
    
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    init_gpio();
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(ret);
    
    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);
    
    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);
    
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(0);
    
    ESP_LOGI(HID_TAG, "Fire TV BLE HID Remote started");
    
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
}