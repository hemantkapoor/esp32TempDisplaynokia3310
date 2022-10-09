/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/timer.h"
#include "pcd8544.h"
#include "thermistor.h"

#define GPIO_LED_PIN    GPIO_NUM_22
#define GPIO_LED_PIN_SEL (1ULL << GPIO_LED_PIN)

#define GPIO_BUTTON_PIN    GPIO_NUM_21
#define GPIO_BUTTOM_PIN_SEL (1ULL << GPIO_BUTTON_PIN)

#define TIMER_DIVIDER   (64)

static void configureGpio(void);
static void handleButtonPress(void* arg);
static void configureTimer(void);
static void timer_group_isr_callback();

static volatile uint32_t currentButtonLevel = 1;
static volatile uint32_t currentTimeMs = 0;

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    configureGpio();
    initThermistor();
    configureTimer();

    pcd8544_config_t config = {
        .spi_host = HSPI_HOST,
        .is_backlight_common_anode = false,
    };
    pcd8544_init(&config);
    pcd8544_clear_display();
    while(true)
     {
        float temperature = getThermistorValue();
        printf("Temperatur = %0.2f", temperature);
    	pcd8544_clear_display();
    	pcd8544_finalize_frame_buf();
    	pcd8544_printf("Temp = %0.2f",temperature);
    	pcd8544_sync_and_gc();

       // gpio_set_level(GPIO_LED_PIN, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //gpio_set_level(GPIO_LED_PIN, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        
    }
}


/*********** Static function definition here ************************/
static void configureGpio(void)
{
    /***********  Configure LED ****************/
    {
        //zero-initialize the config structure.
        gpio_config_t io_conf = {};
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 0;
        //configure GPIO with the given settings
        gpio_config(&io_conf);
         gpio_set_level(GPIO_LED_PIN, 0);
    }


    /***********  Configure Button ****************/
    {
        //zero-initialize the config structure.
        gpio_config_t io_conf = {};
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        //set as output mode
        io_conf.mode = GPIO_MODE_INPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = GPIO_BUTTOM_PIN_SEL;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = true;
        //configure GPIO with the given settings
        gpio_config(&io_conf);    

        ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LOWMED));
        ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_BUTTON_PIN, handleButtonPress,  (void*)GPIO_BUTTON_PIN));

       // gpio_isr_register(handleButtonPress, NULL,ESP_INTR_FLAG_LOWMED,NULL);
    }
}

static void configureTimer(void)
{
    timer_config_t config = 
    {
    .divider = TIMER_DIVIDER,
    .counter_dir = TIMER_COUNT_UP,
    .counter_en = TIMER_PAUSE,
    .alarm_en = TIMER_ALARM_EN,
    .auto_reload = TIMER_AUTORELOAD_DIS
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (TIMER_BASE_CLK / TIMER_DIVIDER));
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0);
    
}

static void IRAM_ATTR handleButtonPress(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;

    if(gpio_num == GPIO_BUTTON_PIN)
    {
        /* If button pressed */
        if(gpio_get_level(gpio_num) == 0)
        {
            configureTimer();
            timer_start(TIMER_GROUP_0, TIMER_0);
        }
        else
        {
            timer_pause(TIMER_GROUP_0, TIMER_0);
        }
    }
}

static void IRAM_ATTR timer_group_isr_callback()
{
    gpio_set_level(GPIO_LED_PIN, currentButtonLevel);
    currentButtonLevel = (currentButtonLevel == 0U? 1U : 0U);    
}