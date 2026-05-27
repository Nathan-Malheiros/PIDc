#include "touch_screen.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

static const char *TAG = "TOUCH";

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

/* -----------------------------------------------------------------------
 * Inicializacao
 * ----------------------------------------------------------------------- */
void touch_screen_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc1_handle));

    ESP_LOGI(TAG, "Touch screen inicializado. Pinos: XP=%d XM=%d YP=%d YM=%d",
             TOUCH_XP_GPIO, TOUCH_XM_GPIO, TOUCH_YP_GPIO, TOUCH_YM_GPIO);
}

/* -----------------------------------------------------------------------
 * Helpers internos
 * ----------------------------------------------------------------------- */
static void pin_as_output(int gpio, int level)
{
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, level);
}

static void pin_as_float(int gpio)
{
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
}

/* Ordena array inteiro in-place (insertion sort — N pequeno, rapido o suficiente) */
static void sort_asc(int *a, int n)
{
    for (int i = 1; i < n; i++) {
        int key = a[i], j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
}

/*
 * Configura pino como ADC e retorna media aparada:
 *   - coleta TOUCH_AVG_SAMPLES leituras com 40 us entre cada uma
 *   - ordena e descarta os 25% menores e 25% maiores (elimina spikes)
 *   - retorna a media do 50% central
 */
static int pin_as_adc_read(int gpio, adc_channel_t ch)
{
    gpio_reset_pin(gpio);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ch, &chan_cfg));

    int samples[TOUCH_AVG_SAMPLES];
    for (int i = 0; i < TOUCH_AVG_SAMPLES; i++) {
        adc_oneshot_read(s_adc1_handle, ch, &samples[i]);
        ets_delay_us(40);
    }

    sort_asc(samples, TOUCH_AVG_SAMPLES);

    int lo = TOUCH_AVG_SAMPLES / 4;       /* 8 para N=32 */
    int hi = TOUCH_AVG_SAMPLES * 3 / 4;  /* 24 para N=32 */
    int32_t sum = 0;
    for (int i = lo; i < hi; i++) sum += samples[i];
    return (int)(sum / (hi - lo));
}

/* -----------------------------------------------------------------------
 * Deteccao de toque — voto majoritario de TOUCH_DETECT_SAMPLES amostras.
 *
 * YP=HIGH, XM=LOW, XP=input_pulldown.
 * Tocado: XP e puxado a HIGH pela camada Y atraves do ponto de contato.
 * ----------------------------------------------------------------------- */
static bool is_touched(void)
{
    gpio_reset_pin(TOUCH_XP_GPIO);
    gpio_reset_pin(TOUCH_XM_GPIO);
    gpio_reset_pin(TOUCH_YP_GPIO);
    gpio_reset_pin(TOUCH_YM_GPIO);

    gpio_set_direction(TOUCH_YP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TOUCH_YP_GPIO, 1);

    gpio_set_direction(TOUCH_XM_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TOUCH_XM_GPIO, 0);

    gpio_set_direction(TOUCH_XP_GPIO, GPIO_MODE_INPUT);
    gpio_pulldown_en(TOUCH_XP_GPIO);

    gpio_set_direction(TOUCH_YM_GPIO, GPIO_MODE_INPUT);

    ets_delay_us(1200);

    int count = 0;
    for (int i = 0; i < TOUCH_DETECT_SAMPLES; i++) {
        if (gpio_get_level(TOUCH_XP_GPIO)) count++;
        ets_delay_us(200);
    }

    gpio_pulldown_dis(TOUCH_XP_GPIO);

    return (count >= TOUCH_DETECT_THRESH);
}

/* -----------------------------------------------------------------------
 * Leitura do eixo X
 * Drive: XP=HIGH, XM=LOW  |  ADC em YP
 * ----------------------------------------------------------------------- */
static int read_raw_x(void)
{
    pin_as_output(TOUCH_XP_GPIO, 1);
    pin_as_output(TOUCH_XM_GPIO, 0);
    pin_as_float(TOUCH_YM_GPIO);

    ets_delay_us(1000);

    return pin_as_adc_read(TOUCH_YP_GPIO, TOUCH_YP_ADC_CH);
}

/* -----------------------------------------------------------------------
 * Leitura do eixo Y
 * Drive: YP=HIGH, YM=LOW  |  ADC em XP
 * ----------------------------------------------------------------------- */
static int read_raw_y(void)
{
    pin_as_output(TOUCH_YP_GPIO, 1);
    pin_as_output(TOUCH_YM_GPIO, 0);
    pin_as_float(TOUCH_XM_GPIO);

    ets_delay_us(1000);

    return pin_as_adc_read(TOUCH_XP_GPIO, TOUCH_XP_ADC_CH);
}

/* -----------------------------------------------------------------------
 * API publica: touch_screen_read
 * ----------------------------------------------------------------------- */
bool touch_screen_read(touch_point_t *point)
{
    if (!is_touched()) {
        point->touched = false;
        return false;
    }

    int raw_x = read_raw_x();
    int raw_y = read_raw_y();

    ESP_LOGD(TAG, "raw_x=%d raw_y=%d", raw_x, raw_y);

    /* Rejeita valores no rail absoluto (pino flutuando = sem contato real) */
    if (raw_x < 30 || raw_x > 4065 || raw_y < 30 || raw_y > 4065) {
        point->touched = false;
        return false;
    }

    float xn = (float)(raw_x - TOUCH_X_RAW_MIN) / (float)(TOUCH_X_RAW_MAX - TOUCH_X_RAW_MIN);
    float yn = (float)(raw_y - TOUCH_Y_RAW_MIN) / (float)(TOUCH_Y_RAW_MAX - TOUCH_Y_RAW_MIN);

    if (xn < 0.0f) xn = 0.0f;
    if (xn > 1.0f) xn = 1.0f;
    if (yn < 0.0f) yn = 0.0f;
    if (yn > 1.0f) yn = 1.0f;

#if TOUCH_X_INVERT
    xn = 1.0f - xn;
#endif
#if TOUCH_Y_INVERT
    yn = 1.0f - yn;
#endif

    point->raw_x = (uint16_t)raw_x;
    point->raw_y = (uint16_t)raw_y;

#if TOUCH_SWAP_XY
    point->x_mm = yn * SCREEN_WIDTH_MM;
    point->y_mm = xn * SCREEN_HEIGHT_MM;
#else
    point->x_mm = xn * SCREEN_WIDTH_MM;
    point->y_mm = yn * SCREEN_HEIGHT_MM;
#endif

    point->touched = true;
    return true;
}
