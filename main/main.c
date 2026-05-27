#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "touch_screen.h"

static const char *TAG = "MAIN";

#define SAMPLE_PERIOD_MS   20

/* Filtro EMA: 0 = sem atualizar, 1 = sem filtro.
 * 0.25 => constante de tempo ~80 ms, suaviza ruido sem atrasar muito. */
#define EMA_ALPHA          0.25f

/* Rejeita leituras que saltam mais que este valor de uma amostra para outra.
 * Catches spikes do ADC sem bloquear movimento real da esfera. */
#define JUMP_MAX_MM        50.0f

void app_main(void)
{
    ESP_LOGI(TAG, "=== Ball Balancer - Rastreador de posicao ===");
    ESP_LOGI(TAG, "Tela resistiva: %d x %d mm", SCREEN_WIDTH_MM, SCREEN_HEIGHT_MM);

    touch_screen_init();

    ESP_LOGI(TAG, "Iniciando rastreamento a %d Hz...", 1000 / SAMPLE_PERIOD_MS);

    touch_point_t point;

    float   filt_x        = -1.0f;   /* -1 = nao inicializado */
    float   filt_y        = -1.0f;
    bool    had_touch     = false;
    int64_t last_touch_us = 0;

    while (1) {
        if (touch_screen_read(&point)) {

            if (filt_x < 0.0f) {
                /* Primeira leitura: inicializa filtro direto */
                filt_x = point.x_mm;
                filt_y = point.y_mm;
            } else {
                float dx = fabsf(point.x_mm - filt_x);
                float dy = fabsf(point.y_mm - filt_y);

                if (dx < JUMP_MAX_MM && dy < JUMP_MAX_MM) {
                    /* Leitura plausivel: aplica EMA */
                    filt_x = EMA_ALPHA * point.x_mm + (1.0f - EMA_ALPHA) * filt_x;
                    filt_y = EMA_ALPHA * point.y_mm + (1.0f - EMA_ALPHA) * filt_y;
                }
                /* Se saltou demais: spike — mantem filt_x/filt_y anterior */
            }

            had_touch     = true;
            last_touch_us = esp_timer_get_time();

            /* CAL: lido pelo calibrate.py (raw sem filtro para calibracao precisa) */
            printf("CAL:%d,%d\n", point.raw_x, point.raw_y);
            printf("X:%.2f,Y:%.2f\n", filt_x, filt_y);

        } else {
            int64_t elapsed_ms = (esp_timer_get_time() - last_touch_us) / 1000;

            if (had_touch && elapsed_ms < TOUCH_HOLD_LAST_MS) {
                /* Dentro da janela de debounce: repete ultima posicao filtrada */
                printf("X:%.2f,Y:%.2f\n", filt_x, filt_y);
            } else {
                printf("NONE\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
