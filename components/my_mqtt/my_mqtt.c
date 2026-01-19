#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "my_mqtt.h"

// URL DO BROKER PARA TESTE
#define MQTT_URL "mqtt://test.mosquitto.org:1883" 

static const char *TAG = "MQTT";

extern SemaphoreHandle_t mqttconnectedSemaphore;
// extern QueueHandle_t filaData; // Comentado pois não existe no main

esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Conectado!");
        xSemaphoreGive(mqttconnectedSemaphore);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT Desconectado");
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "Mensagem publicada, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Dado Recebido!");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        // A lógica de enviar para fila foi removida temporariamente 
        // para evitar erros de compilação, já que o main não processa recepção ainda.
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;

    default:
        ESP_LOGI(TAG, "Outro evento id: %d", event->event_id);
        break;
    }
}

void mqtt_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void mqtt_publish(char *topic, char *msg)
{
    if (client) {
        int msg_id = esp_mqtt_client_publish(client, topic, msg, 0, 1, 0);
        ESP_LOGI(TAG, "Enviando: %s, ID: %d", msg, msg_id);
    }
}

void mqtt_subscribe(char *topic)
{
    if (client) {
        int msg_id = esp_mqtt_client_subscribe(client, topic, 0);
        ESP_LOGI(TAG, "Subscrito em %s, ID: %d", topic, msg_id);
    }
}
