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
#include "esp_gap_bt_api.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/rmt_rx.h"

#define HID_TAG "BLE_HID"
#define HID_DEVICE_NAME "Shamsway"

#define UART_NUM UART_NUM_0
#define UART_BUF_SIZE (1024)

// IR Receiver on GPIO4
#define IR_RX_GPIO GPIO_NUM_4
#define IR_RMT_CLK_RES_HZ 1000000  // 1MHz, 1 tick = 1us

// NEC protocol timing (in microseconds)
#define NEC_LEADING_PULSE_US  9000
#define NEC_LEADING_SPACE_US  4500
#define NEC_REPEAT_SPACE_US   2250
#define NEC_BIT_PULSE_US      560
#define NEC_BIT_ONE_SPACE_US  1690
#define NEC_BIT_ZERO_SPACE_US 560
#define NEC_TOLERANCE_US      200

// IR code mapping for remote
#define IR_CODE_UP        0xF40B4040
#define IR_CODE_DOWN      0xF10E4040
#define IR_CODE_LEFT      0xEF104040
#define IR_CODE_RIGHT     0xEE114040
#define IR_CODE_SELECT    0xF20D4040
#define IR_CODE_BACK      0xBD424040
#define IR_CODE_HOME      0x00000000  // Not mapped
#define IR_CODE_PLAY      0x00000000  // Not mapped
#define IR_CODE_VOL_UP    0x00000000  // Not mapped
#define IR_CODE_VOL_DOWN  0x00000000  // Not mapped

// HID Consumer Control Usage IDs
#define HID_CC_PLAY_PAUSE   0xCD
#define HID_CC_VOLUME_UP    0xE9
#define HID_CC_VOLUME_DOWN  0xEA
#define HID_CC_HOME         0x223

// GATT Service handles
#define GATTS_NUM_HANDLE_HID    8
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 10

// UUID constants as variables
static uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static uint16_t hid_service_uuid = 0x1812;
static uint16_t hid_info_uuid = 0x2A4A;
static uint16_t hid_report_map_uuid = 0x2A4B;
static uint16_t hid_report_uuid = 0x2A4D;
static uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

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
static bool ir_learning_mode = false;

// RMT IR receiver
static rmt_channel_handle_t ir_rx_channel = NULL;
static rmt_symbol_word_t ir_rx_symbols[64];
static rmt_receive_config_t ir_rx_config;
static QueueHandle_t ir_rx_queue = NULL;

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
    0x03, 0x19, 0xC1, 0x03
};

static uint8_t scan_rsp_data[] = {
    0x09, 0x09, 'S', 'h', 'a', 'm', 's', 'w', 'a', 'y'
};

typedef struct {
    rmt_channel_handle_t channel;
    size_t num_symbols;
    rmt_symbol_word_t *symbols;
} ir_rx_event_t;

static bool ir_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    ir_rx_event_t evt = {
        .channel = channel,
        .num_symbols = edata->num_symbols,
        .symbols = edata->received_symbols,
    };
    xQueueSendFromISR(ir_rx_queue, &evt, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static bool nec_check_range(uint32_t duration, uint32_t expected) {
    return (duration >= (expected - NEC_TOLERANCE_US)) && (duration <= (expected + NEC_TOLERANCE_US));
}

static bool nec_decode(rmt_symbol_word_t *symbols, size_t num_symbols, uint32_t *code) {
    if (num_symbols < 34) {
        return false;
    }

    // Check leading pulse
    if (!nec_check_range(symbols[0].duration0, NEC_LEADING_PULSE_US)) {
        return false;
    }

    // Check for repeat code
    if (nec_check_range(symbols[0].duration1, NEC_REPEAT_SPACE_US)) {
        *code = 0xFFFFFFFF;  // Repeat code
        return true;
    }

    // Check leading space
    if (!nec_check_range(symbols[0].duration1, NEC_LEADING_SPACE_US)) {
        return false;
    }

    // Decode 32 bits
    uint32_t decoded = 0;
    for (int i = 0; i < 32; i++) {
        if (!nec_check_range(symbols[1 + i].duration0, NEC_BIT_PULSE_US)) {
            return false;
        }
        if (nec_check_range(symbols[1 + i].duration1, NEC_BIT_ONE_SPACE_US)) {
            decoded |= (1 << i);
        } else if (!nec_check_range(symbols[1 + i].duration1, NEC_BIT_ZERO_SPACE_US)) {
            return false;
        }
    }

    *code = decoded;
    return true;
}

void send_consumer_control(uint16_t usage_id) {
    if (!is_connected || hid_gatts_if == 0) {
        ESP_LOGW(HID_TAG, "Cannot send: not connected");
        return;
    }

    uint8_t report[3];
    report[0] = 0x01;  // Report ID
    report[1] = usage_id & 0xFF;
    report[2] = (usage_id >> 8) & 0xFF;

    ESP_LOGI(HID_TAG, "Sending consumer report: ID=%02X, Usage=%04X", report[0], usage_id);
    esp_err_t ret = esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id,
                                hid_handle_table[6], sizeof(report), report, false);
    ESP_LOGI(HID_TAG, "Send result: %d (handle=%d)", ret, hid_handle_table[6]);

    vTaskDelay(pdMS_TO_TICKS(50));

    report[1] = 0x00;
    report[2] = 0x00;
    esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id,
                                hid_handle_table[6], sizeof(report), report, false);
}

void send_keyboard_key(uint8_t keycode) {
    if (!is_connected || hid_gatts_if == 0) {
        ESP_LOGW(HID_TAG, "Cannot send: not connected");
        return;
    }

    uint8_t report[9] = {0x02, 0x00, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00};
    ESP_LOGI(HID_TAG, "Sending keyboard report: ID=%02X, Key=%02X", report[0], keycode);
    esp_err_t ret = esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id,
                                hid_handle_table[6], sizeof(report), report, false);
    ESP_LOGI(HID_TAG, "Send result: %d (handle=%d)", ret, hid_handle_table[6]);

    vTaskDelay(pdMS_TO_TICKS(50));

    memset(report, 0, sizeof(report));
    report[0] = 0x02;
    esp_ble_gatts_send_indicate(hid_gatts_if, hid_conn_id,
                                hid_handle_table[6], sizeof(report), report, false);
}

void handle_ir_code(uint32_t code) {
    if (code == 0xFFFFFFFF) {
        // Repeat code - ignore or handle repeat
        return;
    }

    if (ir_learning_mode) {
        ESP_LOGI(HID_TAG, "*** IR CODE: 0x%08lX ***", (unsigned long)code);
        ESP_LOGI(HID_TAG, "Add this to the IR_CODE_xxx definitions in main.c");
        return;
    }

    // Map IR codes to actions
    if (code == IR_CODE_UP) {
        ESP_LOGI(HID_TAG, "IR: UP");
        send_keyboard_key(0x52);
    } else if (code == IR_CODE_DOWN) {
        ESP_LOGI(HID_TAG, "IR: DOWN");
        send_keyboard_key(0x51);
    } else if (code == IR_CODE_LEFT) {
        ESP_LOGI(HID_TAG, "IR: LEFT");
        send_keyboard_key(0x50);
    } else if (code == IR_CODE_RIGHT) {
        ESP_LOGI(HID_TAG, "IR: RIGHT");
        send_keyboard_key(0x4F);
    } else if (code == IR_CODE_SELECT) {
        ESP_LOGI(HID_TAG, "IR: SELECT");
        send_keyboard_key(0x28);
    } else if (code == IR_CODE_BACK) {
        ESP_LOGI(HID_TAG, "IR: BACK");
        send_keyboard_key(0x29);
    } else if (code == IR_CODE_HOME) {
        ESP_LOGI(HID_TAG, "IR: HOME");
        send_consumer_control(HID_CC_HOME);
    } else if (code == IR_CODE_PLAY) {
        ESP_LOGI(HID_TAG, "IR: PLAY/PAUSE");
        send_consumer_control(HID_CC_PLAY_PAUSE);
    } else if (code == IR_CODE_VOL_UP) {
        ESP_LOGI(HID_TAG, "IR: VOL UP");
        send_consumer_control(HID_CC_VOLUME_UP);
    } else if (code == IR_CODE_VOL_DOWN) {
        ESP_LOGI(HID_TAG, "IR: VOL DOWN");
        send_consumer_control(HID_CC_VOLUME_DOWN);
    } else {
        ESP_LOGW(HID_TAG, "Unknown IR code: 0x%08lX", (unsigned long)code);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(HID_TAG, "ADV data set, configuring scan response");
            esp_ble_gap_config_scan_rsp_data_raw(scan_rsp_data, sizeof(scan_rsp_data));
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(HID_TAG, "Scan response set, starting advertising");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(HID_TAG, "Advertising start failed");
            } else {
                ESP_LOGI(HID_TAG, "Advertising started successfully");
            }
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI(HID_TAG, "Security request");
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(HID_TAG, "Pairing successful");
            } else {
                ESP_LOGE(HID_TAG, "Pairing failed, status: 0x%x", param->ble_security.auth_cmpl.fail_reason);
            }
            break;
        default:
            break;
    }
}

static const esp_gatts_attr_db_t hid_gatt_db[GATTS_NUM_HANDLE_HID] = {
    // Service Declaration
    [0] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
           ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(hid_service_uuid),
           (uint8_t *)&hid_service_uuid}},

    // HID Information Characteristic Declaration
    [1] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t),
           (uint8_t *)&char_prop_read}},

    // HID Information Characteristic Value
    [2] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_info_uuid,
           ESP_GATT_PERM_READ, sizeof(hid_info_char_val), sizeof(hid_info_char_val),
           hid_info_char_val}},

    // HID Report Map Characteristic Declaration
    [3] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t),
           (uint8_t *)&char_prop_read}},

    // HID Report Map Characteristic Value
    [4] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_map_uuid,
           ESP_GATT_PERM_READ, sizeof(hid_report_map), sizeof(hid_report_map),
           hid_report_map}},

    // HID Report Characteristic Declaration
    [5] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
           ESP_GATT_PERM_READ, sizeof(uint8_t), sizeof(uint8_t),
           (uint8_t *)&char_prop_notify}},

    // HID Report Characteristic Value
    [6] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&hid_report_uuid,
           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, NULL}},

    // HID Report Client Characteristic Configuration Descriptor
    [7] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), 0, NULL}},
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(HID_TAG, "GATT server registered, app_id %04x", param->reg.app_id);

            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(HID_DEVICE_NAME);
            if (set_dev_name_ret) {
                ESP_LOGE(HID_TAG, "Set device name failed, error code = %x", set_dev_name_ret);
            }

            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
            if (raw_adv_ret) {
                ESP_LOGE(HID_TAG, "Config raw adv data failed, error code = %x", raw_adv_ret);
            } else {
                ESP_LOGI(HID_TAG, "Configuring advertising data");
            }

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

void ir_receive_task(void *pvParameters) {
    ir_rx_event_t evt;
    uint32_t ir_code;

    ESP_LOGI(HID_TAG, "IR receiver task started on GPIO%d", IR_RX_GPIO);

    // Start receiving
    ESP_ERROR_CHECK(rmt_receive(ir_rx_channel, ir_rx_symbols, sizeof(ir_rx_symbols), &ir_rx_config));

    while (1) {
        if (xQueueReceive(ir_rx_queue, &evt, portMAX_DELAY) == pdTRUE) {
            if (evt.num_symbols > 0) {
                if (nec_decode(evt.symbols, evt.num_symbols, &ir_code)) {
                    handle_ir_code(ir_code);
                }
            }
            // Re-enable receive
            ESP_ERROR_CHECK(rmt_receive(ir_rx_channel, ir_rx_symbols, sizeof(ir_rx_symbols), &ir_rx_config));
        }
    }
}

void init_ir_receiver(void) {
    ESP_LOGI(HID_TAG, "Initializing IR receiver on GPIO%d", IR_RX_GPIO);

    ir_rx_queue = xQueueCreate(10, sizeof(ir_rx_event_t));

    rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = IR_RX_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RMT_CLK_RES_HZ,
        .mem_block_symbols = 64,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &ir_rx_channel));

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = ir_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(ir_rx_channel, &cbs, NULL));

    ESP_ERROR_CHECK(rmt_enable(ir_rx_channel));

    ir_rx_config.signal_range_min_ns = 1250;      // shortest pulse
    ir_rx_config.signal_range_max_ns = 12000000;  // longest pulse (12ms for leading)
}

void serial_command_task(void *pvParameters) {
    uint8_t data[128];
    int idle_count = 0;

    ESP_LOGI(HID_TAG, "Serial command interface ready");
    ESP_LOGI(HID_TAG, "Commands: u=up, d=down, l=left, r=right, s=select, b=back, h=home, p=play, +=vol up, -=vol down");
    ESP_LOGI(HID_TAG, "Debug: a=restart advertising, i=show info, m=menu, L=toggle IR learning mode");

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(1000));
        if (len > 0) {
            data[len] = 0;
            idle_count = 0;

            for (int i = 0; i < len; i++) {
                char cmd = data[i];

                switch (cmd) {
                    case 'u':
                    case 'U':
                        ESP_LOGI(HID_TAG, "CMD: UP");
                        send_keyboard_key(0x52);
                        break;
                    case 'd':
                    case 'D':
                        ESP_LOGI(HID_TAG, "CMD: DOWN");
                        send_keyboard_key(0x51);
                        break;
                    case 'l':
                        ESP_LOGI(HID_TAG, "CMD: LEFT");
                        send_keyboard_key(0x50);
                        break;
                    case 'r':
                    case 'R':
                        ESP_LOGI(HID_TAG, "CMD: RIGHT");
                        send_keyboard_key(0x4F);
                        break;
                    case 's':
                    case 'S':
                        ESP_LOGI(HID_TAG, "CMD: SELECT");
                        send_keyboard_key(0x28);
                        break;
                    case 'b':
                    case 'B':
                        ESP_LOGI(HID_TAG, "CMD: BACK");
                        send_keyboard_key(0x29);
                        break;
                    case 'h':
                    case 'H':
                        ESP_LOGI(HID_TAG, "CMD: HOME");
                        send_consumer_control(HID_CC_HOME);
                        break;
                    case 'p':
                    case 'P':
                        ESP_LOGI(HID_TAG, "CMD: PLAY/PAUSE");
                        send_consumer_control(HID_CC_PLAY_PAUSE);
                        break;
                    case '+':
                    case '=':
                        ESP_LOGI(HID_TAG, "CMD: VOL UP");
                        send_consumer_control(HID_CC_VOLUME_UP);
                        break;
                    case '-':
                    case '_':
                        ESP_LOGI(HID_TAG, "CMD: VOL DOWN");
                        send_consumer_control(HID_CC_VOLUME_DOWN);
                        break;
                    case 'a':
                    case 'A':
                        ESP_LOGI(HID_TAG, "CMD: Restarting advertising");
                        esp_ble_gap_start_advertising(&adv_params);
                        break;
                    case 'i':
                    case 'I':
                        ESP_LOGI(HID_TAG, "INFO: Connected=%d, GATTS_IF=%d, ConnID=%d",
                                is_connected, hid_gatts_if, hid_conn_id);
                        uint8_t mac[6];
                        esp_read_mac(mac, ESP_MAC_BT);
                        ESP_LOGI(HID_TAG, "INFO: BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                        break;
                    case 'L':
                        ir_learning_mode = !ir_learning_mode;
                        ESP_LOGI(HID_TAG, "IR Learning mode: %s", ir_learning_mode ? "ON" : "OFF");
                        if (ir_learning_mode) {
                            ESP_LOGI(HID_TAG, "Press buttons on your IR remote to see their codes");
                        }
                        break;
                    case 'm':
                    case 'M':
                        ESP_LOGI(HID_TAG, "=== COMMAND MENU ===");
                        ESP_LOGI(HID_TAG, "Navigation: u=UP d=DOWN l=LEFT r=RIGHT");
                        ESP_LOGI(HID_TAG, "Actions: s=SELECT b=BACK h=HOME p=PLAY");
                        ESP_LOGI(HID_TAG, "Volume: +=UP -=DOWN");
                        ESP_LOGI(HID_TAG, "Debug: i=info a=advertise L=IR learn m=menu");
                        break;
                    case '\n':
                    case '\r':
                        break;
                    default:
                        if (cmd >= 32 && cmd < 127) {
                            ESP_LOGW(HID_TAG, "Unknown command: '%c'", cmd);
                        }
                        break;
                }
            }
        } else {
            idle_count++;
            if (idle_count >= 30 && is_connected) {
                ESP_LOGI(HID_TAG, "Type a command: u/d/l/r/s/b/h/p/+/- (or 'm' for menu)");
                idle_count = 0;
            }
        }
    }
}

void app_main(void) {
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_ir_receiver();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));

    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(0);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI(HID_TAG, "Fire TV BLE HID Remote with IR receiver started");
    ESP_LOGI(HID_TAG, "IR receiver on GPIO%d - Press 'L' to enter learning mode", IR_RX_GPIO);

    xTaskCreate(ir_receive_task, "ir_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_command_task, "serial_cmd_task", 4096, NULL, 5, NULL);
}
