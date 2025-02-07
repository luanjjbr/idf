#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Definições dos pinos
#define GPIO_INPUT_PIN 4           // Pino de entrada digital (botão)
#define GPIO_OUTPUT_PIN 2          // Pino de saída digital (LED)
#define ADC_CHANNEL ADC1_CHANNEL_0 // Entrada analógica (GPIO 36)

void configure_pins()
{
    // Configuração do pino de entrada digital (botão)
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << GPIO_INPUT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&input_conf);

    // Configuração do pino de entrada analógica (ADC)
    adc1_config_width(ADC_WIDTH_BIT_12);                     // Configura resolução de 12 bits
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_12); // Escala de 0 a 3.3V

    // Configuração do pino de saída digital (LED)
    gpio_set_direction(GPIO_OUTPUT_PIN, GPIO_MODE_OUTPUT); // Definindo o pino como saída
}

void read_adc_task(void *params)
{
    static const char *tag = "read_adc :";
    while (1)
    {
        // Lê o valor do ADC (0 a 4095)
        int adc_value = adc1_get_raw(ADC_CHANNEL);
        ESP_LOGI(tag, "Valor AD: %d\n", adc_value);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2 segundo
    }
}

void read_gpio_task(void *params)
{
    static const char *tag = "Read_gpio :";
    while (1)
    {
        int button_state = gpio_get_level(GPIO_INPUT_PIN);
        ESP_LOGI(tag, "Estado do botão: %d\n", button_state);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo
    }
}
void blink_led(void *params)
{
    while (1)
    {
        // Código do LED
        gpio_set_level(GPIO_OUTPUT_PIN, 1);    // Acende o LED
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Aguarda 1 segundo
        gpio_set_level(GPIO_OUTPUT_PIN, 0);    // Apaga o LED
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Aguarda 1 segundo
    }
}

void app_main()
{
    // Configura todos os pinos em uma única função
    configure_pins();

    // Cria as tarefas FreeRTOS para leitura do ADC e do GPIO
    xTaskCreate(read_adc_task, "Leitura ADC", 2048, NULL, 1, NULL);
    xTaskCreate(read_gpio_task, "Leitura GPIO", 2048, NULL, 1, NULL);
    xTaskCreate(blink_led, "Blink Led", 2048, NULL, 1, NULL);
}

/**
 * ESP_LOGI("task1", "Leitura do sensor \n"); info
 * ESP_LOGE("task1", "Leitura do sensor \n"); ERRO
 * ESP_LOGW("task1", "Leitura do sensor \n"); Worle
 * ESP_LOGD("task1", "Leitura do sensor \n");
 * ESP_LOGV("task1", "Leitura do sensor \n");
 * ESP_LOGI("task1", "Leitura do sensor");
 * vTaskDelay(1000 / portTICK_PERIOD_MS);
 * xTaskCreatePinnedToCore(&task3, "task3", 2048, "task-3", 1, NULL, 1);
 */