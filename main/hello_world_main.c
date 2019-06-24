/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"

#define GPIO_INPUT  4

static xQueueHandle gpio_queue = NULL;
uint32_t test_cnt = 0;


static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_queue, &gpio_num, NULL);
}

static void gpio_task(void *arg){
    uint32_t gpio_num;

    for(;;){
        if (xQueueReceive(gpio_queue, &gpio_num, portMAX_DELAY)){
            printf("Jebeno\n");
        }
    }
}

void app_main()
{
    gpio_config_t io_conf = {
        //interrupt of rising edge
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << GPIO_INPUT),
        //set as input mode    
        .mode = GPIO_MODE_INPUT,
        //enable pull-up mode
        .pull_up_en = 1
    };

    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INPUT, gpio_isr_handler, (void*) GPIO_INPUT);

    /* Print chip information */
    esp_chip_info_t chip_info;

    esp_chip_info(&chip_info);

    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    gpio_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);
}
