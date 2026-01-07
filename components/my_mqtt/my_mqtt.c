#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "my_mqtt.h"

static const char *TAG = "MQTT";

extern SemaphoreHandle_t mqttconnectedSemaphore;
extern QueueHandle_t filaData;

esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        xSemaphoreGive(mqttconnectedSemaphore);
        break;

    case MQTT_EVENT_DATA:
    {
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\n", event->topic_len, event->topic);
        printf("DATA=%.*s\n", event->data_len, event->data);

        // --- COPIAR DADOS PARA UM BUFFER PRÓPRIO ---
        char buffer[100];
        int len = event->data_len;

        if (len >= sizeof(buffer))
            len = sizeof(buffer) - 1;

        memcpy(buffer, event->data, len);
        buffer[len] = '\0';

        // Enviar para fila
        xQueueSendToBack(filaData, buffer, portMAX_DELAY);
    }
    break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;

    default:
        ESP_LOGI(TAG, "Other event id: %d", event->event_id);
        break;
    }
}

void mqtt_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    esp_mqtt_client_start(client);
}

void mqtt_publish(char *topic, char *msg)
{
    int msg_id = esp_mqtt_client_publish(client, topic, msg, 0, 1, 0);
    ESP_LOGI(TAG, "Mensagem Enviada! ID = %d", msg_id);
}

void mqtt_sbscribe(char *topic)
{
    int msg_id = esp_mqtt_client_subscribe(client, topic, 0);
    ESP_LOGI(TAG, "Subscrito no Tópico %s! ID = %d", topic, msg_id);
}
