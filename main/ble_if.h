#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "ble_odometer.h"
#include <esp_gatt_defs.h>

#ifndef BLE_IF_H
#define BLE_IF_H

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
#define GATTS_TABLE_TAG             "GATTS_SPP_DEMO"
#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x69
#define SAMPLE_DEVICE_NAME          "BIKEMETER"
#define SPP_SVC_INST_ID	            0
/// Characteristic UUID
// TODO
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      ESP_GATT_UUID_CSC_FEATURE
// TODO
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       ESP_GATT_UUID_CSC_MEASUREMENT


extern uint16_t spp_handle_table[SPP_IDX_NB];
extern uint16_t spp_conn_id;
extern esp_gatt_if_t spp_gatts_if;

/// SPP Service
// static const uint16_t spp_service_uuid = 0xABF0;
static const uint16_t spp_service_uuid = ESP_GATT_UUID_CYCLING_SPEED_CADENCE_SVC;


/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR|ESP_GATT_CHAR_PROP_BIT_READ;

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};


uint8_t find_char_and_desr_index(uint16_t handle);
bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data);
void free_write_buffer(void);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#endif