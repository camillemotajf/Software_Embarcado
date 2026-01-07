#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include "ssd1306.h"
#include "freertos/semphr.h"
// #include "mqtt_client.h"
#include "my_mqtt.h"
#include <message_buffer.h>
#include <cJSON.h>

// ================= DEFINIÇÕES DE PINOS =================

// Ultrassônico (HC-SR04)
#define TRIG_PIN    5
#define ECHO_PIN    18
#define DIST_LIMIT_CM  20.0f  // Distância de parada (cm)

// Joystick
#define ADC_JOY_X   ADC1_CHANNEL_6   // GPIO34
#define JOY_BUTTON  15               // Botão do joystick

// Sensores Infravermelhos
#define IR_CH_1     ADC1_CHANNEL_0   // GPIO 36
#define IR_CH_2     ADC1_CHANNEL_3   // GPIO 39
#define IR_CH_3     ADC1_CHANNEL_4   // GPIO 32
#define IR_CH_4     ADC1_CHANNEL_5   // GPIO 33
#define NUM_SENSORS 4

// OLED
#define SDA_PIN 21
#define SCL_PIN 22
#define RST_PIN -1

// Motor
#define MOTOR_IN1  26
#define MOTOR_IN2  27
#define MOTOR_ENA  14

// PWM
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY  5000

#define TAG "ROBOT_V2"

// MQTT
SemaphoreHandle_t wificonnectedSemaphore;
SemaphoreHandle_t mqttconnectedSemaphore;

MessageBufferHandle_t buffer_MQTT;

// ================= ESTRUTURAS =================

typedef enum {
    MODE_AUTO_IR = 0,
    MODE_MANUAL_JOY = 1
} robot_mode_t;

typedef struct {
    float x;
    robot_mode_t mode;
    bool blocked;       // Flag de bloqueio
    float distance_cm;  // Valor para debug no OLED
} joy_t;

typedef struct {
    int min_val[NUM_SENSORS];
    int max_val[NUM_SENSORS];
    int cal_val[NUM_SENSORS];
} ir_sensors_t;

typedef struct {
    robot_mode_t mode;
    bool blocked;
    float distance;
} mqtt_data_t;

// ================= VARIÁVEIS GLOBAIS =================

static QueueHandle_t joyQueueControl;
static QueueHandle_t joyQueueOled;
static QueueHandle_t mqttQueue;

static ir_sensors_t ir_data;
static const adc1_channel_t ir_channels[NUM_SENSORS] =
    {IR_CH_1, IR_CH_2, IR_CH_3, IR_CH_4};

// ================= HARDWARE =================

void motor_init(void) {
    gpio_set_direction(MOTOR_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_IN2, GPIO_MODE_OUTPUT);

    gpio_set_direction(JOY_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOY_BUTTON, GPIO_PULLUP_ONLY);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .gpio_num = MOTOR_ENA,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void ultrasonic_init(void) {
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(TRIG_PIN, 0);
}

void set_motor_state(int dir, int duty) {
    if (dir > 0) {
        gpio_set_level(MOTOR_IN1, 1);
        gpio_set_level(MOTOR_IN2, 0);
    } else if (dir < 0) {
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 1);
    } else {
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 0);
        duty = 0;
    }
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// Leitura mais robusta do Ultrassônico
float get_ultrasonic_cm(void) {
    // 1. Trigger
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    // 2. Espera ECHO ficar HIGH (Timeout 20ms)
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - start) > 20000) return -1.0; 
    }

    // 3. Mede o tempo em HIGH (Timeout 25ms ~ 4m)
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - echo_start) > 25000) break; 
    }
    int64_t echo_end = esp_timer_get_time();

    // 4. Cálculo
    float distance = (float)(echo_end - echo_start) / 58.0f;
    
    // 5. Filtro básico de hardware
    if (distance > 400.0f) return 400.0f;
    if (distance < 2.0f && distance > 0.0f) return 2.0f; // Mínimo físico do sensor
    
    return distance;
}

// ================= SENSORES IR =================

void calibrate_sensors_phase(void) {
    ESP_LOGI(TAG, "Calibrando sensores IR...");
    for (int i = 0; i < NUM_SENSORS; i++) {
        ir_data.min_val[i] = 4095;
        ir_data.max_val[i] = 0;
    }

    // Leitura rápida para calibração
    for (int k = 0; k < 200; k++) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            int v = adc1_get_raw(ir_channels[i]);
            if (v < ir_data.min_val[i]) ir_data.min_val[i] = v;
            if (v > ir_data.max_val[i]) ir_data.max_val[i] = v;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Evita divisão por zero
    for (int i = 0; i < NUM_SENSORS; i++)
        if (ir_data.max_val[i] <= ir_data.min_val[i])
            ir_data.max_val[i] = ir_data.min_val[i] + 1;

    ESP_LOGI(TAG, "Calibrado.");
}

void read_calibrated_sensors(void) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = adc1_get_raw(ir_channels[i]);
        int val = ((raw - ir_data.min_val[i]) * 1000) / (ir_data.max_val[i] - ir_data.min_val[i]);
        
        if (val < 0) val = 0;
        if (val > 1000) val = 1000;
        ir_data.cal_val[i] = val;
    }
}

// ================= LÓGICA DE CONTROLE =================

float get_manual_command(void) {
    int raw = adc1_get_raw(ADC_JOY_X);
    int center = 2048; 
    int deadzone = 200; // Aumentei deadzone para evitar drift

    int dx = raw - center;
    if (abs(dx) < deadzone) return 0.0f;

    float norm = (float)dx / (4095 - center);
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;

    return norm;
}

float get_auto_command(void) {
    int s1 = ir_data.cal_val[0];
    int s2 = ir_data.cal_val[1];
    int s3 = ir_data.cal_val[2];
    int s4 = ir_data.cal_val[3];

    int th = 500;

    // Lógica simples de seguidor de linha
    if (s1 > th) return -1.0f; // Curva forte esq
    if (s4 > th) return  1.0f; // Curva forte dir
    if (s2 > th) return -0.5f; // Curva suave esq
    if (s3 > th) return  0.5f; // Curva suave dir

    return 0.0f; // Reto
}

// ================= TASKS =================

void master_control_task(void *pv) {
    calibrate_sensors_phase();

    robot_mode_t mode = MODE_AUTO_IR;
    joy_t payload;
    int last_btn = 1;
    float last_valid_dist = 100.0f; // Guarda a ultima distancia valida

    mqtt_data_t mqtt_msg;

    while (1) {
        // 1. Troca de Modo
        int btn = gpio_get_level(JOY_BUTTON);
        if (last_btn && !btn) {
            mode = (mode == MODE_AUTO_IR) ? MODE_MANUAL_JOY : MODE_AUTO_IR;
            vTaskDelay(pdMS_TO_TICKS(300)); // Debounce
        }
        last_btn = btn;

        // 2. Leitura Sensores
        read_calibrated_sensors();
        float dist_raw = get_ultrasonic_cm();

        // 3. Filtragem Simples da Distância
        // Se der -1 (timeout), mantém o último valor válido para não desbloquear falsamente
        // Se for > 0 e < 400, atualiza
        if (dist_raw > 0.1f && dist_raw <= 400.0f) {
            last_valid_dist = dist_raw;
        }

        // 4. Lógica de Bloqueio
        bool is_blocked = false;
        // Só bloqueia se a leitura for VÁLIDA e menor que o limite
        if (last_valid_dist < DIST_LIMIT_CM && last_valid_dist > 1.0f) {
            is_blocked = true;
        }

        // 5. Cálculo do Comando Motor
        float cmd = 0.0f;
        if (is_blocked) {
            cmd = 0.0f; // Prioridade total para parada
        } else {
            cmd = (mode == MODE_MANUAL_JOY) ? get_manual_command() : get_auto_command();
        }

        // 6. Envio para Display e Motor
        payload.x = cmd;
        payload.mode = mode;
        payload.blocked = is_blocked;
        payload.distance_cm = last_valid_dist; // Envia para o OLED ver

        xQueueSend(joyQueueControl, &payload, 0);
        xQueueOverwrite(joyQueueOled, &payload);


        mqtt_msg.mode = mode;
        mqtt_msg.blocked = is_blocked;
        mqtt_msg.distance = last_valid_dist;

        xQueueOverwrite(mqttQueue, &mqtt_msg);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void motor_task(void *pv) {
    joy_t joy;
    // Ajuste de potência
    const int DUTY_LOW = 4800;  // Um pouco mais força para arrancar
    const int DUTY_HIGH = 8191;

    while (1) {
        if (xQueueReceive(joyQueueControl, &joy, portMAX_DELAY)) {
            
            // Segurança redundante no motor task
            if (joy.blocked) {
                set_motor_state(0, 0);
                continue; 
            }

            if (joy.x > 0.1f)
                set_motor_state(1, joy.x > 0.7f ? DUTY_HIGH : DUTY_LOW);
            else if (joy.x < -0.1f)
                set_motor_state(-1, joy.x < -0.7f ? DUTY_HIGH : DUTY_LOW);
            else
                set_motor_state(0, 0);
        }
    }
}

void oled_task(void *pv) {
    SSD1306_t oled;
    joy_t joy;
    char buf[20];

    i2c_master_init(&oled, SDA_PIN, SCL_PIN, RST_PIN);
    ssd1306_init(&oled, 128, 64);
    ssd1306_clear_screen(&oled, false);

    while (1) {
        if (xQueueReceive(joyQueueOled, &joy, portMAX_DELAY)) {

            // LINHA 0: MODO
            ssd1306_display_text(&oled, 0,
                joy.mode == MODE_AUTO_IR ? "MODO: AUTO (IR)" : "MODO: MANUAL",
                16, false);

            // LINHA 1: DISTÂNCIA (NOVO REQUISITO)
            if (joy.distance_cm > 300) {
                 snprintf(buf, sizeof(buf), "DIST: >300 cm");
            } else {
                 snprintf(buf, sizeof(buf), "DIST: %3.1f cm", joy.distance_cm);
            }
            ssd1306_display_text(&oled, 1, buf, strlen(buf), false);

            // LINHA 2: SENSORES IR (Compactado para caber)
            // Mostra S1 e S4 (Extremidades) que são os mais importantes
            snprintf(buf, sizeof(buf), "IR: %3d  %3d", 
                     ir_data.cal_val[0], ir_data.cal_val[3]);
            ssd1306_display_text(&oled, 2, buf, strlen(buf), false);

            // LINHA 3: STATUS FINAL
            if (joy.blocked) {
                ssd1306_display_text(&oled, 3, "BLOQUEADO!    ", 16, true); // Invertido para destaque
            } else {
                if (joy.x > 0.1)
                    ssd1306_display_text(&oled, 3, "MOTOR: >>>    ", 16, false);
                else if (joy.x < -0.1)
                    ssd1306_display_text(&oled, 3, "MOTOR: <<<    ", 16, false);
                else
                    ssd1306_display_text(&oled, 3, "MOTOR: PARADO ", 16, false);
            }
        }
    }
}

// ================= MQTT =================

void wifiConnected(void *params)
{
    while (1)
    {
        if (xSemaphoreTake(wificonnectedSemaphore, portMAX_DELAY))
        {
            ESP_LOGI("WIFI", "WiFi conectado! Iniciando MQTT...");
            mqtt_start();
        }
    }
}

void send_broker_infos(const char mode, const char status)
{
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "device", "esp32");
    cJSON_AddStringToObject(root, "mode", mode);
    cJSON_AddStringToObject(root, "status", status);

    char *json_str = cJSON_PrintUnformatted(root);

    mqtt_publish("esp32/motor", json_str);

    ESP_LOGI("MQTT", "JSON enviado: %s", json_str);

    cJSON_Delete(root);
    free(json_str);
}

void comunicacao_broker(void *pv)
{
    mqtt_data_t data;

    // Espera MQTT conectar
    xSemaphoreTake(mqttconnectedSemaphore, portMAX_DELAY);

    while (1) {
        if (xQueueReceive(mqttQueue, &data, portMAX_DELAY)) {

            cJSON *root = cJSON_CreateObject();

            cJSON_AddStringToObject(root, "device", "esp32");
            cJSON_AddStringToObject(root, "mode",
                data.mode == MODE_AUTO_IR ? "auto" : "manual");
            cJSON_AddBoolToObject(root, "blocked", data.blocked);
            cJSON_AddNumberToObject(root, "distance_cm", data.distance);

            char *json = cJSON_PrintUnformatted(root);

            mqtt_publish("esp32/robot/status", json);

            cJSON_Delete(root);
            free(json);
        }
    }
}


// ================= MAIN =================

void app_main(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_JOY_X, ADC_ATTEN_DB_12);

    adc1_config_channel_atten(IR_CH_1, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(IR_CH_2, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(IR_CH_3, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(IR_CH_4, ADC_ATTEN_DB_12);

    motor_init();
    ultrasonic_init();

    joyQueueControl = xQueueCreate(10, sizeof(joy_t));
    joyQueueOled    = xQueueCreate(1,  sizeof(joy_t));
    mqttQueue = xQueueCreate(5, sizeof(mqtt_data_t));

    // Aumentei stack do Master pois agora tem lógica de float e timer
    xTaskCreate(master_control_task, "master", 4096, NULL, 5, NULL);
    xTaskCreate(motor_task,          "motor",  2048, NULL, 4, NULL);
    xTaskCreate(oled_task,           "oled",   4096, NULL, 1, NULL);

    // Semáforos
    wificonnectedSemaphore = xSemaphoreCreateBinary();
    mqttconnectedSemaphore = xSemaphoreCreateBinary();

    // Buffer MQTT
    buffer_MQTT = xMessageBufferCreate(100);

    // Start Wi-Fi
    wifi_start();

    // Tasks
    xTaskCreate(wifiConnected,       "Conexao MQTT",     4096, NULL, 2, NULL);
    xTaskCreate(comunicacao_broker,  "ComBroker",        4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "Sistema iniciado com v2.");
}
