/*
 * main.c
 *
 * Program to read ESP32 ADC input.
 */

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/clk_tree_ll.h"
#include <stdio.h>

static const char *TAG = "esp-adc";

/* GPIO items. */

#define LED_GPIO 2
#define MOTOR_GPIO 16
#define TEST_GPIO 17
#define SYNC_GPIO 23

static void configure_out_pin(gpio_num_t gpio) {
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
}

static void toggle_out_pin(gpio_num_t gpio, bool *state) {
    *state = !(*state);
    gpio_set_level(gpio, *state);
}

static void configure_interrupt_pin(gpio_num_t gpio, gpio_isr_t isr) {

    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio, GPIO_FLOATING);
    gpio_set_intr_type(gpio, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio, isr, NULL);
}

static adc_continuous_handle_t adc = NULL;

static TaskHandle_t main_task;

/* ADC items.  I want to collect 1000 frames (samples) of one channel each
conversion cycle. */

#define SAMPLES 1000
#define FRAME_SZ (SAMPLES * SOC_ADC_DIGI_RESULT_BYTES)
#define BUF_SZ (FRAME_SZ)

static void configure_adc(adc_continuous_handle_t  *adc,
                          adc_continuous_callback_t isr) {

    /* For the NODEMCU ESP32-S device:

       ADC1_CH6	VDET_1	GPIO34	J1.5
       ADC1_CH7	VDET_2	GPIO35	J1.6
    */
    static const adc_channel_t channels[1] = {ADC_CHANNEL_6};

    /* This is a bit confusing.  The ADC converts at a MINIMUM rate of
       20 KHz, but it can stash away a series of conversions before
       bothering the CPU, I guess by DMA (although I don't have to
       configure a DMA channel).  Here we specify the number of
       conversions in one of these blocks, which the documentation
       calls frames.  Notice that the frame size below is in bytes,
       but each conversion result (a single conversion of a single
       channel) requires several bytes, as determined by the type of
       ESP32 we have (in the soc_caps.h file). */

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = BUF_SZ,
        .conv_frame_size    = FRAME_SZ,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, adc));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num                                         = 1;
    for (int i = 0; i < 1; i++) {
        uint8_t ch               = channels[i] & 0x7;
        adc_pattern[i].atten     = ADC_ATTEN_DB_11;
        adc_pattern[i].channel   = ch;
        adc_pattern[i].unit      = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i,
                 adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(*adc, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {.on_conv_done = isr};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(*adc, &cbs, NULL));
}

static bool check_valid_data(const adc_digi_output_data_t *data) {
    if (data->type1.channel >= SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
        return false;
    }
    return true;
}

static uint32_t               conversion_count = 0;
static adc_digi_output_data_t result[SAMPLES];

static void IRAM_ATTR sync_handler(void *arg) {

    static bool led_is_on = false;
    static int  count     = 0;

    if (count++ % 20 == 0) {
        adc_continuous_start(adc);
        toggle_out_pin(LED_GPIO, &led_is_on);
    }
}

static bool IRAM_ATTR conv_handler(adc_continuous_handle_t          adc,
                                   const adc_continuous_evt_data_t *edata,
                                   void                            *udata) {

    BaseType_t must_yield = pdFALSE;
    conversion_count++;
    vTaskNotifyGiveFromISR(main_task, &must_yield);
    return must_yield == pdTRUE;
}

void app_main(void) {

    ESP_LOGI(TAG, "Start main.");

    bool test_state = false;

    main_task = xTaskGetCurrentTaskHandle();

    configure_out_pin(LED_GPIO);
    configure_out_pin(TEST_GPIO);
    configure_adc(&adc, conv_handler);
    configure_interrupt_pin(SYNC_GPIO, sync_handler);

    /* Figure out the CPU frequency.  This currently reports 160 MHz, as set in
    the sdkconfig file. */

    soc_cpu_clk_src_t clk_src   = clk_ll_cpu_get_src();
    uint32_t          clk_speed = clk_ll_cpu_get_freq_mhz_from_pll();

    printf("Clock source is %d at speed %ld.\n", clk_src, clk_speed);

    /* Loop forever blinking the LED. */

    while (1) {

        esp_err_t err        = ESP_OK;
        uint32_t  bytes_read = 0;

        /* Wait for new ADC values. */

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        err =
            adc_continuous_read(adc, (uint8_t *)result, BUF_SZ, &bytes_read, 0);
        adc_continuous_stop(adc);

        if (err == ESP_OK) {

            toggle_out_pin(TEST_GPIO, &test_state);

            /* Log data... */
            /* Dump out the entire result buffer. */

            for (int i = 0; i < SAMPLES; i++) {
                adc_digi_output_data_t *p = &result[i];
                if (check_valid_data(p)) {
                    printf("%d\t%d\n", i, p->type1.data);
                } else {
                    ESP_LOGI(TAG, "Invalid data");
                }
            }
        } else {
            ESP_LOGI(TAG, "Got %d, %" PRIu32 " bytes", err, bytes_read);
        }
    }
}
