/* MQTT over SSL Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/param.h>

//#include "protocol_examples_common.h"
//esp lib
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_crc.h"
#include "esp_smartconfig.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
//#include "esp_now.h"

#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"

//freeRTOS lib
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

//user lib
//#include "espnow_config.h"
#include "mqtt_config.h"
#include "gpio_config.h"


static const char *TAG = "MQTTS_EXAMPLE";


extern uint8_t GPIO_OUTPUT_IO[4];
uint8_t TOUCH_PAD[4] = {4, 5, 6, 7};



//Smartconfig
#define ESPTOUCH_TIMEOUT_S     20
static const int CONNECTED_BIT      = BIT0;
static const int ESPTOUCH_DONE_BIT  = BIT1;

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)


static uint16_t s_pad_init_val[TOUCH_PAD_MAX];
static bool s_pad_state[TOUCH_PAD_MAX] = {false};
static bool s_pad_change[TOUCH_PAD_MAX] = {false};

static bool led_state[4] = {false};


//Queqe ,event handle
QueueHandle_t xQueuePublish;


//NVS
#define STORAGE_NAMESPACE "storage"
static nvs_handle_t nvs_wifi_config_handle;


static EventGroupHandle_t s_wifi_event_group = NULL;
//static EventGroupHandle_t led_status_event_group = NULL;


/* Handler wifi envent*/
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Start wifi station mode");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wifi disconnect \n Retry connect...");
        gpio_set_level(RED_LED_STATUS, 1);
        gpio_set_level(GREEN_LED_STATUS, 0);
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
        gpio_set_level(RED_LED_STATUS, 0);
        gpio_set_level(GREEN_LED_STATUS, 1);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };
        uint8_t rvd_data[33] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_wifi_config_handle);

        nvs_set_str(nvs_wifi_config_handle, "wifi_ssid", (char*)wifi_config.sta.ssid);
        nvs_set_str(nvs_wifi_config_handle, "wifi_password", (char*)wifi_config.sta.password);

        nvs_commit(nvs_wifi_config_handle);

        // Close
        nvs_close(nvs_wifi_config_handle);

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }


        ESP_ERROR_CHECK( esp_wifi_disconnect());
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        esp_wifi_connect();

    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}


/* WiFi should start before using ESPNOW */
static void initialise_wifi(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    static bool initialized = false;
    if (initialized) {
        return;
    }

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    //esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    //assert(ap_netif);
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg));
    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &event_handler, NULL));
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    esp_err_t err;
    wifi_config_t wifi_config;

    bzero(&wifi_config, sizeof(wifi_config_t));

    nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_wifi_config_handle);
 
    // Read the size of memory space required for blob
    size_t wifi_ssid_size = sizeof(wifi_config.sta.ssid);
    size_t wifi_password_size = sizeof(wifi_config.sta.password);

    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK( esp_wifi_start());

    err = nvs_get_str(nvs_wifi_config_handle, "wifi_ssid", (char*)wifi_config.sta.ssid, &wifi_ssid_size);
    err = nvs_get_str(nvs_wifi_config_handle, "wifi_password", (char*)wifi_config.sta.password, &wifi_password_size);

    ESP_LOGI(TAG, "SSID:%s", wifi_config.sta.ssid);
    ESP_LOGI(TAG, "PASSWORD:%s", wifi_config.sta.password);


    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Load configured wifi complete");
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        if(esp_wifi_connect() == ESP_OK){
            gpio_set_level(RED_LED_STATUS, 0);
            gpio_set_level(GREEN_LED_STATUS, 1);
        }

    }else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Wifi configured not found");
    }else{
        ESP_LOGI(TAG, "NVS error");
    }

    nvs_close(nvs_wifi_config_handle);

    uint8_t sta_mac[6] = {0};
    //uint8_t ap_mac[6] = {0};
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    //esp_wifi_get_mac(ESP_IF_WIFI_AP, ap_mac);
    ESP_LOGI(TAG, "sta_mac:" MACSTR ,MAC2STR(sta_mac));
    //ESP_LOGI(TAG, "ap_mac:" MACSTR ,MAC2STR(ap_mac));

    initialized = true;
}

//Gpio function connect wifi
static void btn_smartconfig_task(void* arg)
{
    const uint32_t TIME_LONG_CLICK_MS = 2000;
    uint32_t counter = 0;
    static bool smartconfig_done = false;
    for(;;) {
        if(gpio_get_level(BTN_SMARTCONFIG) == 0) {
            vTaskDelay(20 / portTICK_RATE_MS);
            if(gpio_get_level(BTN_SMARTCONFIG) == 0) {
                counter ++;  
            } else {
                counter = 0;
            }
            if((counter*20) >= TIME_LONG_CLICK_MS){
                counter = 0;
                EventBits_t uxBits;
                ESP_LOGI(TAG, "Start smartconfig ...");
                ESP_ERROR_CHECK( esp_wifi_disconnect());
                
                ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_V2) );
                esp_esptouch_set_timeout(ESPTOUCH_TIMEOUT_S);
                smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
                ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );

                smartconfig_done = false;
                while (smartconfig_done == false) {
                    uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
                    if(uxBits & CONNECTED_BIT) {
                        ESP_LOGI(TAG, "WiFi Connected to ap");
                        gpio_set_level(RED_LED_STATUS, 0);
                        gpio_set_level(GREEN_LED_STATUS, 1);
                    }
                    if(uxBits & ESPTOUCH_DONE_BIT) {
                        ESP_LOGI(TAG, "smartconfig over");
                        esp_smartconfig_stop();

                        smartconfig_done = true;
                    }
                }
            }
        }
        vTaskDelay(20 / portTICK_RATE_MS);
    }
}

static void mygpio_init(void)
{

    GPIO_OUTPUT_IO[0] = 18;
    GPIO_OUTPUT_IO[1] = 19;
    GPIO_OUTPUT_IO[2] = 21;
    GPIO_OUTPUT_IO[3] = 23;
    unsigned long long  GPIO_OUTPUT_PIN_SEL = 0;
    GPIO_OUTPUT_PIN_SEL = ((1<<GPIO_OUTPUT_IO[0])+(1<<GPIO_OUTPUT_IO[1])+(1<<GPIO_OUTPUT_IO[2])+(1<<GPIO_OUTPUT_IO[3])+(1<<GREEN_LED_STATUS)+(1<<RED_LED_STATUS));
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;                  //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                        //set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;             //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pull_down_en = 0;                               //disable pull-down mode
    io_conf.pull_up_en = 0;                                 //disable pull-up mode
    gpio_config(&io_conf);                                  //configure GPIO with the given settings


    io_conf.intr_type = GPIO_INTR_DISABLE;                  //disable interrupt
    io_conf.mode = GPIO_MODE_INPUT;                         //set as input mode
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;              //bit mask of the pins, use GPIO4/5 here
    io_conf.pull_up_en = 1;                                 //enable pull-up mode
    gpio_config(&io_conf);

}

/*
  Read values sensed at all available touch pads.
 Print out values in a loop on a serial monitor.
 */
static void tp_example_read_task(void *pvParameter)
{
    uint16_t touch_filter_value;
    for (int i = 0; i < 4; i++) {
        touch_pad_read_filtered(TOUCH_PAD[i], &s_pad_init_val[TOUCH_PAD[i]]);
    }

    while (1) {
        for (int i = 0; i < 4; i++) {
            touch_pad_read_filtered(TOUCH_PAD[i], &touch_filter_value);

            if(touch_filter_value < s_pad_init_val[TOUCH_PAD[i]]*9/10){
                if(s_pad_state[TOUCH_PAD[i]] == true){
                    s_pad_change[TOUCH_PAD[i]] = false;
                } else {
                    s_pad_change[TOUCH_PAD[i]] = true;
                    gpio_set_level(GPIO_OUTPUT_IO[i], led_state[i]);
                    ESP_LOGI(TAG, "Device %d change",i);
                    s_pad_change[TOUCH_PAD[i]] = false;
                    led_state[i] = !led_state[i];

                }
                s_pad_state[TOUCH_PAD[i]] = true;

            } else {
                s_pad_state[TOUCH_PAD[i]] = false;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// static void controls_device_task(void *pvParameter){

//     while(1){
//         for (int i = TOUCH_PAD_0; i <= TOUCH_PAD_3; i++) {
//             if(s_pad_change[i] == true){
//                 switch (i){
//                     case TOUCH_PAD_0:
//                         gpio_set_level(GPIO_OUTPUT_IO_0, led_state[i]);
//                         ESP_LOGI(TAG, "Device 1 change");
//                         break;
//                     case TOUCH_PAD_1:
//                         gpio_set_level(GPIO_OUTPUT_IO_1, 1);
//                         ESP_LOGI(TAG, "Device 2 change");
//                         break;
//                     case TOUCH_PAD_2:
//                         gpio_set_level(GPIO_OUTPUT_IO_2, 1);
//                         ESP_LOGI(TAG, "Device 3 change");
//                         break;
//                     case TOUCH_PAD_3:
//                         gpio_set_level(GPIO_OUTPUT_IO_3, 1);
//                         ESP_LOGI(TAG, "Device 4 change");
//                         break;
//                 }
//                 s_pad_change[i] = false;
//             }
//         }
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//     }
    
// }


static void tp_example_touch_pad_init(void)
{

    for (int i = 0; i < 4; i++) {
        touch_pad_config(TOUCH_PAD[i], TOUCH_THRESH_NO_USE);
    }

}


void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);


        // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    mygpio_init();
    initialise_wifi();

    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
        ESP_ERROR_CHECK(touch_pad_init());
    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V5);
    tp_example_touch_pad_init();
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    

    xQueuePublish = xQueueCreate( 10, sizeof(MQTT_t));
    //example_espnow_init();
    xTaskCreate(btn_smartconfig_task, "btn_smartconfig_task", 2048, NULL, 4, NULL);
    // Start a task to show what pads have been touched
    xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
    //xTaskCreate(&controls_device_task, "controls_device_task", 2048, NULL, 5, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", 2048*4, NULL, 4, NULL);
    
}
