#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"

// Definições dos pinos
#define GPIO_INPUT_PIN 4           // Pino de entrada digital (botão)
#define GPIO_OUTPUT_PIN 2          // Pino de saída digital (LED)
#define ADC_CHANNEL ADC1_CHANNEL_5 // Entrada analógica (GPIO 35)

// Definindo a fila
QueueHandle_t fila;

SemaphoreHandle_t semaforo;
// Definindo a notificação
TaskHandle_t task1_handle = NULL;
TaskHandle_t task2_handle = NULL;

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

void read_adc_task(void *params)
{
    static const char *tag = "read_adc :";
    while (1)
    {
        // Lê o valor do ADC (0 a 4095)
        int adc_value = adc1_get_raw(ADC_CHANNEL);
        ESP_LOGE(tag, "Valor AD: %d\n", adc_value);
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

void tarefa1(void *pvParameter)
{
    static const char *tag = "tarefa1 :";
    int valor = 0;

    while (1)
    {
        valor = (rand() % 50);
        // Envia o valor para a fila
        if (xQueueSend(fila, &valor, portMAX_DELAY))
        {
            ESP_LOGE(tag, "Enviando valor: %d\n", valor);
            // Notifica a tarefa 2 de que há um valor disponível
            xTaskNotifyGive(task2_handle);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay de 1 segundo
    }
}
void tarefa2(void *pvParameter)
{
    static const char *tag = "tarefa2 :";
    int valorRecebido;

    while (1)
    {
        // Espera pela notificação da tarefa 1
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Recebe o valor da fila
        if (xQueueReceive(fila, &valorRecebido, portMAX_DELAY))
        {
            ESP_LOGW(tag, "Recebido valor: %d\n", valorRecebido);
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay de 2 segundo
    }
}
void tarefa3(void *param)
{
    while (1)
    {
        // Tenta pegar o semáforo
        if (xSemaphoreTake(semaforo, portMAX_DELAY))
        {
            printf("Tarefa 3 :: Ligando LED\n");
            gpio_set_level(GPIO_OUTPUT_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Mantém ligado por 1s
            gpio_set_level(GPIO_OUTPUT_PIN, 0);
            printf("Tarefa 3 :: Desligando LED\n");
            xSemaphoreGive(semaforo); // Libera o semáforo
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Espera antes de tentar novamente
    }
}

void tarefa4(void *param)
{
    while (1)
    {
        if (xSemaphoreTake(semaforo, portMAX_DELAY))
        { // Tenta pegar o semáforo
            printf("[Tarefa 4 :: Ligando LED\n");
            gpio_set_level(GPIO_OUTPUT_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Mantém ligado por 1s
            gpio_set_level(GPIO_OUTPUT_PIN, 0);
            printf("Tarefa 4 :: Desligando LED\n");
            xSemaphoreGive(semaforo); // Libera o semáforo
        }
        vTaskDelay(pdMS_TO_TICKS(3000)); // Espera antes de tentar novamente
    }
}

void app_main()
{
    // Configura todos os pinos em uma única função
    configure_pins();

    // Cria as tarefas FreeRTOS para leitura do ADC e do GPIO
    // xTaskCreate(blink_led, "Blink Led", 2048, NULL, 1, NULL);
    xTaskCreate(read_adc_task, "Leitura ADC", 2048, NULL, 1, NULL);
    xTaskCreate(read_gpio_task, "Leitura GPIO", 2048, NULL, 1, NULL);

    fila = xQueueCreate(5, sizeof(int));

    // Cria as tarefas
    xTaskCreate(tarefa2, "Tarefa 2", 2048, NULL, 1, &task2_handle);
    xTaskCreate(tarefa1, "Tarefa 1", 2048, NULL, 1, &task1_handle);

    // Cria o semáforo
    semaforo = xSemaphoreCreateBinary();
    xSemaphoreGive(semaforo); // Inicializa disponível

    // Cria as tarefas
    xTaskCreatePinnedToCore(tarefa3, "Tarefa3", 2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(tarefa4, "Tarefa4", 2048, NULL, 1, NULL, 1);
}