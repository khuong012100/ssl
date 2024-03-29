/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

//C lib
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

//freeRTOS lib
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

//esp lib
#include "esp_log.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "lwip/mem.h"
#include "driver/gpio.h"

//user lib
#include "mqtt_config.h"
#include "gpio_config.h"


static const char *TAG = "mqtt_task";


uint8_t GPIO_OUTPUT_IO[4];
bool led_state[4];
const char *controls_device[4];
const char *status_device[4];


extern QueueHandle_t xQueuePublish;
static EventGroupHandle_t mqtt_status_event_group;

//#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipseprojects_io_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
//#else
//extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
//#endif
//extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");



/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
   ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
   esp_mqtt_event_handle_t event = event_data;
   esp_mqtt_client_handle_t client = event->client;
   int msg_id;
   switch ((esp_mqtt_event_id_t)event_id) {
   case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
      for(int i = 0; i < 4; i++){
         msg_id = esp_mqtt_client_subscribe(client, controls_device[i], 0);
         ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
         xEventGroupSetBits(mqtt_status_event_group, MQTT_CONNECTED_BIT);
      }
      //msg_id = esp_mqtt_client_unsubscribe(client, "topic/qos1");
      //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
      break;
   case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      xEventGroupClearBits(mqtt_status_event_group, MQTT_CONNECTED_BIT);
      break;

   case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
      //msg_id = esp_mqtt_client_publish(client, "topic/qos0", "data", 0, 0, 0);
      //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
      break;
   case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      break;
    case MQTT_EVENT_DATA:
      ESP_LOGI(TAG, "MQTT_EVENT_DATA");
      printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
      printf("DATA=%.*s\r\n", event->data_len, event->data);
      for(int i = 0; i < 4; i++){
         if (strncmp(event->topic, controls_device[i], event->topic_len) == 0) {
			   if (strncmp(event->data, "on", event->data_len) == 0) {
				   ESP_LOGI(TAG, "Turn on device %d", i);
				   esp_mqtt_client_publish(client, status_device[i], "on", 0, 0, 0);
				   gpio_set_level(GPIO_OUTPUT_IO[i], 1);
               led_state[i] = true;
     		   } else if(strncmp(event->data, "off", event->data_len) == 0) {
				   ESP_LOGI(TAG, "Turn off device %d", i);
				   esp_mqtt_client_publish(client, status_device[i], "off", 0, 0, 0);
				   gpio_set_level(GPIO_OUTPUT_IO[i], 0);
               led_state[i] = false;
     		   }
         }
      }
      break;
   case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
         ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
         ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
         ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                  strerror(event->error_handle->esp_transport_sock_errno));
      } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
         ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
      } else {
         ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
      }
      break;
   default:
      ESP_LOGI(TAG, "Other event id:%d", event->event_id);
      break;
    }
}

void mqtt_task(void *pvParameters)
{

	mqtt_status_event_group = xEventGroupCreate();
	configASSERT( mqtt_status_event_group );

	const esp_mqtt_client_config_t mqtt_cfg = {
      .uri = CONFIG_BROKER_URI,
      .cert_pem = (const char *)mqtt_eclipseprojects_io_pem_start,
   };

   esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
   xEventGroupClearBits(mqtt_status_event_group, MQTT_CONNECTED_BIT);
   /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
   esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
   esp_mqtt_client_start(client);
   xEventGroupWaitBits(mqtt_status_event_group, MQTT_CONNECTED_BIT, false, true, portMAX_DELAY);

   ESP_LOGI(TAG, "Connect to MQTT Server");

	MQTT_t mqttBuf;
	while (1) {
		xQueueReceive(xQueuePublish, &mqttBuf, portMAX_DELAY);
		ESP_LOGI(TAG, "type=%d", mqttBuf.topic_type);

		EventBits_t EventBits = xEventGroupGetBits(mqtt_status_event_group);
		ESP_LOGI(TAG, "EventBits=%x", EventBits);
		if (EventBits & MQTT_CONNECTED_BIT) {
			if (mqttBuf.topic_type == PUBLISH) {
				ESP_LOGI(TAG, "topic=%s data=%s", mqttBuf.topic, mqttBuf.data);
				int msg_id = esp_mqtt_client_publish(client, mqttBuf.topic, mqttBuf.data, 0, 1, 0);
				ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
			} else if (mqttBuf.topic_type == STOP) {
				ESP_LOGI(TAG, "Task Stop");
				break;
			}
		} else {
			ESP_LOGW(TAG, "Disconnect to MQTT Server. Skip to send");
		}
	}

	ESP_LOGI(TAG, "Task Delete");
	esp_mqtt_client_stop(client);
	vTaskDelete(NULL);
}

