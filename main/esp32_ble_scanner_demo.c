/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/



/****************************************************************************
*
* This demo showcases BLE GATT client. It can scan BLE devices and connect to one device.
* Run the gatt_server demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"

#include "list.h"

#define GATTC_TAG "GATTC_DEMO"
#define TAG "UART_DEMO"
#define REMOTE_SERVICE_UUID        0x1214
#define REMOTE_NOTIFY_CHAR_UUID    0x19B10001E8F2537E4F6CD104768A1214
//  19b1  01e8f2537e4f6cd1 4768a1214
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

static const char remote_device_name[] = "LED";
static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t* char_elem_result = NULL;
static esp_gattc_descr_elem_t* descr_elem_result = NULL;

// UART
#define BUF_SIZE        1024
#define RD_BUF_SIZE     (BUF_SIZE)
#define QUEUE_SIZE      20
#define PATTERN_CHR_NUM 3

static QueueHandle_t uart0_queue;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param);

/* Menu state */
static volatile uint8_t menu_state = 0;
struct ble_scan_result_evt_param* connected_device = NULL;

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {
        .uuid128[0] = 0x14,
        .uuid128[1] = 0x12,
        .uuid128[2] = 0x8a,
        .uuid128[3] = 0x76,
        .uuid128[4] = 0x04,
        .uuid128[5] = 0xd1,
        .uuid128[6] = 0x6c,
        .uuid128[7] = 0x4f,
        .uuid128[8] = 0x7e,
        .uuid128[9] = 0x53,
        .uuid128[10] = 0xf2,
        .uuid128[11] = 0xe8,
        .uuid128[12] = 0x01,
        .uuid128[13] = 0x00,
        .uuid128[14] = 0xB1,
        .uuid128[15] = 0x19,
    },
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};




/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param) {
    esp_ble_gattc_cb_param_t* p_data = (esp_ble_gattc_cb_param_t*)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret) {
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
        if (mtu_ret) {
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        ESP_LOGI(GATTC_TAG, "Search service with filter 0x%x", remote_filter_service_uuid.uuid.uuid16);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);

        ESP_LOGI(GATTC_TAG, "Search all services");
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);

        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        ESP_LOGI(GATTC_TAG, "service uuid with length %d: 0x%x", p_data->search_res.srvc_id.uuid.len, p_data->search_res.srvc_id.uuid.uuid.uuid16);
        if (p_data->search_res.srvc_id.uuid.len >= ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        }
        else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        }
        else {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server) {
            uint16_t count = 0;
            uint16_t char_count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                p_data->search_cmpl.conn_id,
                ESP_GATT_DB_CHARACTERISTIC,
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                INVALID_HANDLE,
                &count);
            if (status != ESP_GATT_OK) {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            ESP_LOGI(GATTC_TAG, "Attributes count %d", count);
            ESP_LOGI(GATTC_TAG, "\n");

            if (count > 0) {
                char_elem_result = (esp_gattc_char_elem_t*)malloc(sizeof(esp_gattc_char_elem_t) * count);

                status = esp_ble_gattc_get_all_char(gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    char_elem_result,
                    &count,
                    0);

                for (int i = 0; i < count; i++) {

                    ESP_LOGI(GATTC_TAG, "Characteristics count %d. Value: ", count);
                    esp_log_buffer_hex(GATTC_TAG, char_elem_result[i].uuid.uuid.uuid128, char_elem_result[i].uuid.len);
                    ESP_LOGI(GATTC_TAG, "\n");
                }

                /* free char_elem_result */
                free(char_elem_result);
            }

            if (count > 0) {
                char_elem_result = (esp_gattc_char_elem_t*)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result) {
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }
                else {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                        p_data->search_cmpl.conn_id,
                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                        remote_filter_char_uuid,
                        char_elem_result,
                        &count);
                    if (status != ESP_GATT_OK) {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    ESP_LOGI(GATTC_TAG, "Properties %d", char_elem_result[0].properties);

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }
            else {
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }
        else {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                ESP_GATT_DB_DESCRIPTOR,
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                &count);
            if (ret_status != ESP_GATT_OK) {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0) {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result) {
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                }
                else {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                        p_data->reg_for_notify.handle,
                        notify_descr_uuid,
                        descr_elem_result,
                        &count);
                    if (ret_status != ESP_GATT_OK) {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
                        ESP_LOGI(GATTC_TAG, "Write characteristics description");
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                            descr_elem_result[0].handle,
                            sizeof(notify_en),
                            (uint8_t*)&notify_en,
                            ESP_GATT_WRITE_TYPE_RSP,
                            ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK) {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else {
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify) {
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }
        else {
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success ");
        // uint8_t write_char_data[35];
        uint8_t write_char_data = 0x03;
        // for (int i = 0; i < sizeof(write_char_data); ++i)
        // {
        //     write_char_data[i] = i % 256;
        // }
        esp_ble_gattc_write_char(gattc_if,
            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
            gl_profile_tab[PROFILE_A_APP_ID].char_handle,
            sizeof(write_char_data),
            &write_char_data,
            ESP_GATT_WRITE_TYPE_RSP,
            ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write char success ");
        ESP_LOGI(GATTC_TAG, "\n");

        // ESP_LOGI(GATTC_TAG, "Sleeping for 10 s and then write again");
        // vTaskDelay(10000 / portTICK_PERIOD_MS);

        // uint8_t char_d = esp_random() & 0x07;
        // ESP_LOGI(GATTC_TAG, "Write char %x", char_d);

        // esp_ble_gattc_write_char(gattc_if,
        //     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
        //     gl_profile_tab[PROFILE_A_APP_ID].char_handle,
        //     sizeof(char_d),
        //     &char_d,
        //     ESP_GATT_WRITE_TYPE_RSP,
        //     ESP_GATT_AUTH_REQ_NONE);

        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        ESP_LOGI(GATTC_TAG, "Default case %d\n", event);
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    uint8_t* adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        ESP_LOGI(GATTC_TAG, "BLE_SCAN_PARAM_SET_COMPLETE_EVT: Scan params set!");
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_SCAN_RESULT_EVT");

        esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:

            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);

            /*
            struct scan_result_t{
                // esp_bd_addr_t bda
                // name
                // name_len
            } scan_result;

            struct scan_result_list_t{
                scan_result_t data;
                scan_result_t *pNext;
            } scan_result_list;

            add_scan_result_to_list(scan_result scan_result, adv_name, adv_name_len);

            */

            if (adv_name_len > 0) {
                // esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
                // ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
                // ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
                // esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
                adv_name[adv_name_len - 1] = '\0';
                add_scan_rest_to_list(&(scan_result->scan_rst), adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
                if (scan_result->scan_rst.adv_data_len > 0) {
                    ESP_LOGI(GATTC_TAG, "adv data:");
                    esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
                }
                if (scan_result->scan_rst.scan_rsp_len > 0) {
                    ESP_LOGI(GATTC_TAG, "scan resp:");
                    esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
                }
#endif
                // ESP_LOGI(GATTC_TAG, "\n");
            }

            // Show List of scanned bluetooth devices

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char*)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
            param->update_conn_params.status,
            param->update_conn_params.min_int,
            param->update_conn_params.max_int,
            param->update_conn_params.conn_int,
            param->update_conn_params.latency,
            param->update_conn_params.timeout);
        break;
    default:
        ESP_LOGI(GATTC_TAG, "Default case %d", event);
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param) {
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                param->reg.app_id,
                param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void vTimerCallbackScanCompleted(xTimerHandle pxTimer) {
    ESP_LOGI(GATTC_TAG, "Scan is  done");
    display_scan_results();
    menu_state = 1;
}

static void handle_user_input(const char* input, const uint8_t* tmp) {
    ESP_LOGI(TAG, "I am receiving user input: %s %x %d", input, tmp[0], menu_state);

    if (menu_state == 0) {
        if (input[0] == '1') {
            // Start scanning
            ESP_LOGI(GATTC_TAG, "Start scanning");
            uint32_t duration = 30;
            esp_ble_gap_start_scanning(duration);

            // Create timer callback to notify if scanning is over.
            TimerHandle_t timerHandle;
            timerHandle = xTimerCreate(
                "BLEScanTimer",
                pdMS_TO_TICKS(35000),
                pdFALSE, // auto reload
                (void*)0,
                vTimerCallbackScanCompleted
            );
            if (timerHandle == NULL) {
                ESP_LOGE(GATTC_TAG, "Unable to create timer.");
            }

            if (xTimerStart(timerHandle, 0) != pdPASS) {
                ESP_LOGE(GATTC_TAG, "Unable to start timer.");
            }
        }
        else {
            // Do nothing
        }
    }
    else if (menu_state == 1) {

        struct ble_scan_result_evt_param* result = NULL;

        find_device_by_index(input[0] - '0', &result);

        if (result != NULL) {
            if (connect == false) {
                ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                connect = true;
                esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                    result->bda,
                    result->ble_addr_type,
                    true);
                menu_state = 2;
                connected_device = result;
            }
        }
        else {
            ESP_LOGI(GATTC_TAG, "Not found");
        }
    }
    else if (menu_state == 2) {
        if (input[0] == '9') {
            ESP_LOGI(GATTC_TAG, "Disconnect remote device.");
            esp_ble_gap_disconnect(connected_device->bda);
            menu_state = 1;

            display_scan_results();
        }
        else {
            // Set the LED value
            ESP_LOGI(GATTC_TAG, "Write char %x", input[0]);
            uint8_t char_d = input[0] - '0';

            esp_ble_gattc_write_char(
                gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                sizeof(char_d),
                &char_d,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE);

        }
    }
}

// Main menu task
void main_menu_task(void* pvParameter) {
    printf("ESP32 BLE Scanner Demo\n");

    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*)malloc(RD_BUF_SIZE);

    while (1) {
        if (xQueueReceive(uart0_queue, (void*)&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_0);
            switch (event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
            case UART_DATA:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]:");
                handle_user_input((const char*)dtmp, dtmp);
                uart_write_bytes(UART_NUM_0, (const char*)dtmp, event.size);
                break;
                //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(uart0_queue);
                break;
                //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_0);
                xQueueReset(uart0_queue);
                break;
                //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
                //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
                //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
                //UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(UART_NUM_0, &buffered_size);
                int pos = uart_pattern_pop_pos(UART_NUM_0);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(UART_NUM_0);
                }
                else {
                    uart_read_bytes(UART_NUM_0, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(UART_NUM_0, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
                //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
}

void app_main(void) {
    // Initialize NVS. (ESP32 modules run code from an external flash).
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    // Setting UART Communication
    ESP_LOGI(TAG, "Setting UART Communication");
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // Install UART driver
    uart_driver_install(UART_NUM_0, BUF_SIZE, BUF_SIZE, QUEUE_SIZE, &uart0_queue, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_enable_pattern_det_baud_intr(UART_NUM_0, '+', 3, 9, 0, 0);
    uart_pattern_queue_reset(UART_NUM_0, 20);

    // Create a task waiting for user input
    xTaskCreate(&main_menu_task, "main_menu", 2048, NULL, 5, NULL);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // Bluedroid -> Just a bluetooth stack.
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

}

