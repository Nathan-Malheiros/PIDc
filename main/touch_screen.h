#pragma once

#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Dimensoes fisicas da tela resistiva (em milimetros)
 * ----------------------------------------------------------------------- */
#define SCREEN_WIDTH_MM    187
#define SCREEN_HEIGHT_MM   141

/* -----------------------------------------------------------------------
 * Pinos do ESP32-S3 conectados a tela resistiva 4 fios
 *
 * Ligacao padrao tela resistiva 4 fios:
 *
 *   Fio / FPC  Sinal  GPIO ESP32-S3  ADC
 *   --------  ------  -----------  ----------
 *   Pin 1 (ou A)  X+   GPIO 4      ADC1_CH3
 *   Pin 2 (ou B)  X-   GPIO 5      ADC1_CH4
 *   Pin 3 (ou C)  Y+   GPIO 6      ADC1_CH5
 *   Pin 4 (ou D)  Y-   GPIO 7      ADC1_CH6
 *
 * NOTA: a ordem dos pinos no conector FPC varia por fabricante.
 * Se os eixos ficarem trocados ou invertidos, troque XP<->YP ou
 * inverta os calculos de normalizacao nos defines abaixo.
 * ----------------------------------------------------------------------- */
#define TOUCH_XP_GPIO      4       /* X+  - saida HIGH ao ler X, ADC ao ler Y */
#define TOUCH_XM_GPIO      5       /* X-  - saida LOW  ao ler X               */
#define TOUCH_YP_GPIO      6       /* Y+  - saida HIGH ao ler Y, ADC ao ler X */
#define TOUCH_YM_GPIO      7       /* Y-  - saida LOW  ao ler Y               */

#define TOUCH_XP_ADC_UNIT  ADC_UNIT_1
#define TOUCH_XP_ADC_CH    ADC_CHANNEL_3   /* GPIO 4 */
#define TOUCH_YP_ADC_UNIT  ADC_UNIT_1
#define TOUCH_YP_ADC_CH    ADC_CHANNEL_5   /* GPIO 6 */

/* -----------------------------------------------------------------------
 * Calibracao: valores ADC brutos nas bordas da tela.
 * Use tools/calibrate.py para obter os valores corretos do seu painel.
 * ----------------------------------------------------------------------- */
#define TOUCH_X_RAW_MIN    3006
#define TOUCH_X_RAW_MAX    4052
#define TOUCH_Y_RAW_MIN    2687
#define TOUCH_Y_RAW_MAX    3924

/* Inversao de eixo — mude para 1 se o eixo estiver espelhado.
 * Use tools/calibrate.py: ele detecta e imprime o valor correto. */
#define TOUCH_X_INVERT     1   /* 1 = inverter X (biblioteca original inverte) */
#define TOUCH_Y_INVERT     1   /* 1 = inverter Y                                */
#define TOUCH_SWAP_XY      0   /* 1 = trocar eixos X e Y                        */

/* Amostras ADC por eixo — media aparada (descarta 25% superior e inferior) */
#define TOUCH_AVG_SAMPLES     32

/* Amostras de deteccao de toque e limiar de maioria (4 de 6) */
#define TOUCH_DETECT_SAMPLES  6
#define TOUCH_DETECT_THRESH   4

/* ms sem toque antes de declarar NONE (debounce de saida) */
#define TOUCH_HOLD_LAST_MS    300

/* -----------------------------------------------------------------------
 * Estrutura de retorno da leitura
 * ----------------------------------------------------------------------- */
typedef struct {
    uint16_t raw_x;   /* valor ADC bruto (0-4095) no eixo X */
    uint16_t raw_y;   /* valor ADC bruto (0-4095) no eixo Y */
    float    x_mm;    /* posicao em mm na largura (0 a 181) */
    float    y_mm;    /* posicao em mm na altura  (0 a 141) */
    bool     touched; /* true se a esfera esta em contato   */
} touch_point_t;

/* -----------------------------------------------------------------------
 * API publica
 * ----------------------------------------------------------------------- */
void touch_screen_init(void);
bool touch_screen_read(touch_point_t *point);
