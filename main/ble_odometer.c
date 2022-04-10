/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/timer.h"

#include "ble_if.h"
#include "ble_odometer.h"

#define GPIO_INPUT  16
/*********************************************************************************/
void timer_func(void);

/*********************************************************************************/

// Timer
static const uint32_t timer_divider = 8;
#define TIMER_SCALE     (TIMER_BASE_CLK / timer_divider)
static const timer_idx_t timer_idx = TIMER_0;
static const timer_group_t timer_group  = TIMER_GROUP_1;

// gpio
static xQueueHandle gpio_queue = NULL;
// Debouncing,
static double last_time;
// Time in seconds.
static const double debounce_time = 0.5;
static uint32_t delta;

static
void
format(uint32_t revs, double time, uint8_t *dest_buff)
{
    uint8_t buff[7];
    
    uint16_t csc_time = time * 1024;

    buff[0] = 0x01;

    buff[1] = revs && 0xFF;
    buff[2] = (revs >> 8) && 0xFF;
    buff[3] = (revs >> 16) && 0xFF;
    buff[4] = (revs >> 24) && 0xFF;

    buff[5] = csc_time;
    buff[6] = csc_time >> 8;

    memcpy(dest_buff, buff, 7);

    return ;
}

static void IRAM_ATTR gpio_isr_handler(void *arg){
    double cur_time;
    static uint32_t revs = 0;
    uint8_t buff[7];

    timer_get_counter_time_sec(timer_group, timer_idx, &cur_time);

    if ((cur_time - last_time) > debounce_time){
        revs++;
        // TODO: remove this shit.
        delta = cur_time - last_time;
        format(revs, cur_time, buff);

        xQueueSendFromISR(gpio_queue, &buff, NULL);
    }

    last_time = cur_time;  
}

// struct _send_buf {
//     wheel_
// }

static void gpio_task(void *arg){
    uint32_t gpio_num;
    static double last_time = 0;
    char str_buffer[15] = {0};
    uint8_t send_buff[7];

    // GPIO stuff ------> move this to module
    gpio_config_t io_conf = {
        //interrupt of rising edge
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << GPIO_INPUT),
        //set as input mode    
        .mode = GPIO_MODE_INPUT,
        //enable pull-up mode
        .pull_up_en = 0,
        .pull_down_en = 0
    };

    gpio_config(&io_conf);    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INPUT, gpio_isr_handler, (void*) GPIO_INPUT);

    timer_func();

    for(;;){
        if (xQueueReceive(gpio_queue, &send_buff, portMAX_DELAY)){
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 7, send_buff, false);
        }
    }
}

// Timer
void timer_func(void){
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = timer_divider;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 0;
    
    timer_init(timer_group, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    // timer_set_alarm_value(TIMER_GROUP_1, timer_idx, timer_interval_sec * TIMER_SCALE);
    
    timer_start(timer_group, timer_idx);
}


void app_main()
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Bluetooth stuff
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    gpio_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
}
