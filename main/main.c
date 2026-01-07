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
#include "ssd1306.h"

// ================= DEFINIÇÕES DE PINOS =================

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

#define TAG "ROBOT_MASTER"

// ================= ESTRUTURAS =================

typedef enum {
    MODE_AUTO_IR = 0,
    MODE_MANUAL_JOY = 1
} robot_mode_t;

typedef struct {
    float x;
    robot_mode_t mode;
} joy_t;

typedef struct {
    int min_val[NUM_SENSORS];
    int max_val[NUM_SENSORS];
    int cal_val[NUM_SENSORS];
} ir_sensors_t;

// ================= VARIÁVEIS GLOBAIS =================

static QueueHandle_t joyQueueControl;
static QueueHandle_t joyQueueOled;

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

// ================= SENSORES IR =================

void calibrate_sensors_phase(void) {

    ESP_LOGI(TAG, "Calibrando sensores IR...");

    for (int i = 0; i < NUM_SENSORS; i++) {
        ir_data.min_val[i] = 4095;
        ir_data.max_val[i] = 0;
    }

    for (int k = 0; k < 250; k++) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            int v = adc1_get_raw(ir_channels[i]);
            if (v < ir_data.min_val[i]) ir_data.min_val[i] = v;
            if (v > ir_data.max_val[i]) ir_data.max_val[i] = v;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for (int i = 0; i < NUM_SENSORS; i++)
        if (ir_data.max_val[i] <= ir_data.min_val[i])
            ir_data.max_val[i] = ir_data.min_val[i] + 1;

    ESP_LOGI(TAG, "Calibracao finalizada.");
}

void read_calibrated_sensors(void) {

    for (int i = 0; i < NUM_SENSORS; i++) {
        int raw = adc1_get_raw(ir_channels[i]);
        int min = ir_data.min_val[i];
        int max = ir_data.max_val[i];

        int val = ((raw - min) * 1000) / (max - min);
        if (val < 0) val = 0;
        if (val > 1000) val = 1000;

        ir_data.cal_val[i] = val;
    }
}

// ================= CONTROLE =================

float get_manual_command(void) {

    int raw = adc1_get_raw(ADC_JOY_X);
    int center = 2048;
    int deadzone = 150;

    int dx = raw - center;
    if (abs(dx) < deadzone) return 0.0f;

    float norm = (float)dx / (4095 - center);
    if (norm > 1) norm = 1;
    if (norm < -1) norm = -1;

    return norm;
}

float get_auto_command(void) {

    int s1 = ir_data.cal_val[0];
    int s2 = ir_data.cal_val[1];
    int s3 = ir_data.cal_val[2];
    int s4 = ir_data.cal_val[3];

    int th = 500;

    if (s1 > th) return -1.0f;
    if (s4 > th) return  1.0f;
    if (s2 > th) return -0.5f;
    if (s3 > th) return  0.5f;

    return 0.0f;
}

// ================= TASKS =================

void master_control_task(void *pv) {

    calibrate_sensors_phase();

    robot_mode_t mode = MODE_AUTO_IR;
    joy_t payload;
    int last_btn = 1;

    while (1) {

        int btn = gpio_get_level(JOY_BUTTON);
        if (last_btn && !btn) {
            mode = (mode == MODE_AUTO_IR) ? MODE_MANUAL_JOY : MODE_AUTO_IR;
            ESP_LOGW(TAG, "Modo alterado");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        last_btn = btn;

        read_calibrated_sensors();

        payload.x = (mode == MODE_MANUAL_JOY) ?
                    get_manual_command() :
                    get_auto_command();

        payload.mode = mode;

        xQueueSend(joyQueueControl, &payload, 0);
        xQueueOverwrite(joyQueueOled, &payload);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void motor_task(void *pv) {

    joy_t joy;
    const int DUTY_LOW = 4500;
    const int DUTY_HIGH = 8191;

    while (1) {
        if (xQueueReceive(joyQueueControl, &joy, portMAX_DELAY)) {

            if (joy.x > 0.0f)
                set_motor_state(1, joy.x > 0.6f ? DUTY_HIGH : DUTY_LOW);
            else if (joy.x < 0.0f)
                set_motor_state(-1, joy.x < -0.6f ? DUTY_HIGH : DUTY_LOW);
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

            ssd1306_display_text(&oled, 0,
                joy.mode == MODE_AUTO_IR ? "MODO: AUTO (IR)" : "MODO: MANUAL",
                16, false);

            snprintf(buf, sizeof(buf),
                     "S1:%4d S2:%4d",
                     ir_data.cal_val[0], ir_data.cal_val[1]);
            ssd1306_display_text(&oled, 1, buf, strlen(buf), false);

            snprintf(buf, sizeof(buf),
                     "S3:%4d S4:%4d",
                     ir_data.cal_val[2], ir_data.cal_val[3]);
            ssd1306_display_text(&oled, 2, buf, strlen(buf), false);

            if (joy.x > 0.1)
                ssd1306_display_text(&oled, 3, "MOTOR: DIREITA", 16, false);
            else if (joy.x < -0.1)
                ssd1306_display_text(&oled, 3, "MOTOR: ESQUERDA", 16, false);
            else
                ssd1306_display_text(&oled, 3, "MOTOR: PARADO", 16, false);
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

    joyQueueControl = xQueueCreate(10, sizeof(joy_t));
    joyQueueOled    = xQueueCreate(1,  sizeof(joy_t));

    xTaskCreate(master_control_task, "master", 4096, NULL, 5, NULL);
    xTaskCreate(motor_task,          "motor",  2048, NULL, 4, NULL);
    xTaskCreate(oled_task,           "oled",   4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "Sistema iniciado.");
}