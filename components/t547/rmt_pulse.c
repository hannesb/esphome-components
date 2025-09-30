#include "rmt_pulse.h"

#include <esp_idf_version.h>

#if ESP_IDF_VERSION_MAJOR >= 5
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_types.h>
#else
#include <driver/rmt.h>
#include <esp_intr_alloc.h>
#if ESP_IDF_VERSION_MAJOR >= 4
#include <hal/rmt_ll.h>
#include <soc/rmt_struct.h>
#endif
#endif

#include <esp_err.h>
#include <string.h>

#if ESP_IDF_VERSION_MAJOR >= 5
static rmt_channel_handle_t s_rmt_channel = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;
static rmt_transmit_config_t s_tx_config = {
    .loop_count = 0,
};
#else
static intr_handle_t gRMT_intr_handle = NULL;
static rmt_config_t row_rmt_config;
#endif

static volatile bool rmt_tx_done = true;

#if ESP_IDF_VERSION_MAJOR >= 5
static bool IRAM_ATTR rmt_tx_done_callback(rmt_channel_handle_t channel,
                                           const rmt_tx_done_event_data_t *edata,
                                           void *user_ctx)
{
    (void)channel;
    (void)edata;
    (void)user_ctx;
    rmt_tx_done = true;
    return false;
}
#else
/**
 * Remote peripheral interrupt. Used to signal when transmission is done.
 */
static void IRAM_ATTR rmt_interrupt_handler(void *arg)
{
    (void)arg;
    rmt_tx_done = true;
    RMT.int_clr.val = RMT.int_st.val;
}
#endif

void rmt_pulse_init(gpio_num_t pin)
{
#if ESP_IDF_VERSION_MAJOR >= 5
    rmt_tx_channel_config_t channel_config = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 0.1us per tick
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };

    if (rmt_new_tx_channel(&channel_config, &s_rmt_channel) != ESP_OK) {
        s_rmt_channel = NULL;
        return;
    }

    if (rmt_enable(s_rmt_channel) != ESP_OK) {
        s_rmt_channel = NULL;
        return;
    }

    rmt_copy_encoder_config_t encoder_config;
    memset(&encoder_config, 0, sizeof(encoder_config));
    if (rmt_new_copy_encoder(&encoder_config, &s_copy_encoder) != ESP_OK) {
        s_copy_encoder = NULL;
        return;
    }

    rmt_tx_event_callbacks_t callbacks = {
        .on_trans_done = rmt_tx_done_callback,
    };
    rmt_tx_register_event_callbacks(s_rmt_channel, &callbacks, NULL);
    rmt_tx_done = true;
#else
    row_rmt_config.rmt_mode = RMT_MODE_TX;
    // currently hardcoded: use channel 0
    row_rmt_config.channel = RMT_CHANNEL_1;

    row_rmt_config.gpio_num = pin;
    row_rmt_config.mem_block_num = 2;

    // Divide 80MHz APB Clock by 8 -> .1us resolution delay
    row_rmt_config.clk_div = 8;

    row_rmt_config.tx_config.loop_en = false;
    row_rmt_config.tx_config.carrier_en = false;
    row_rmt_config.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    row_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    row_rmt_config.tx_config.idle_output_en = true;

#if ESP_IDF_VERSION_MAJOR >= 4
    rmt_isr_register(rmt_interrupt_handler, NULL,
                     ESP_INTR_FLAG_LEVEL3, &gRMT_intr_handle);
#else
    esp_intr_alloc(ETS_RMT_INTR_SOURCE, ESP_INTR_FLAG_LEVEL3,
                   rmt_interrupt_handler, NULL, &gRMT_intr_handle);
#endif

    rmt_config(&row_rmt_config);
#if ESP_IDF_VERSION_MAJOR >= 4
    rmt_ll_enable_tx_end_interrupt(&RMT, row_rmt_config.channel, true);
#else
    rmt_set_tx_intr_en(row_rmt_config.channel, true);
#endif
    rmt_tx_done = true;
#endif
}

void pulse_ckv_ticks(uint16_t high_time_ticks, uint16_t low_time_ticks, bool wait)
{
#if ESP_IDF_VERSION_MAJOR >= 5
    if (s_rmt_channel == NULL || s_copy_encoder == NULL) {
        return;
    }

    while (!rmt_tx_done) {
    }

    rmt_symbol_word_t symbol = {
        .level0 = 1,
        .duration0 = high_time_ticks ? high_time_ticks : low_time_ticks,
        .level1 = 0,
        .duration1 = high_time_ticks ? low_time_ticks : 0,
    };

    if (high_time_ticks == 0) {
        symbol.duration1 = 0;
    }

    rmt_tx_done = false;
    if (rmt_transmit(s_rmt_channel, s_copy_encoder, &symbol, sizeof(symbol), &s_tx_config) != ESP_OK) {
        rmt_tx_done = true;
        return;
    }

    if (wait) {
        while (!rmt_tx_done) {
        }
    }
#else
    while (!rmt_tx_done) {
    }
    volatile rmt_item32_t *rmt_mem_ptr =
        &(RMTMEM.chan[row_rmt_config.channel].data32[0]);
    if (high_time_ticks > 0) {
        rmt_mem_ptr->level0 = 1;
        rmt_mem_ptr->duration0 = high_time_ticks;
        rmt_mem_ptr->level1 = 0;
        rmt_mem_ptr->duration1 = low_time_ticks;
    } else {
        rmt_mem_ptr->level0 = 1;
        rmt_mem_ptr->duration0 = low_time_ticks;
        rmt_mem_ptr->level1 = 0;
        rmt_mem_ptr->duration1 = 0;
    }
    RMTMEM.chan[row_rmt_config.channel].data32[1].val = 0;
    rmt_tx_done = false;
    RMT.conf_ch[row_rmt_config.channel].conf1.mem_rd_rst = 1;
    RMT.conf_ch[row_rmt_config.channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
    RMT.conf_ch[row_rmt_config.channel].conf1.tx_start = 1;
    if (wait) {
        while (!rmt_tx_done) {
        }
    }
#endif
}

void pulse_ckv_us(uint16_t high_time_us, uint16_t low_time_us,
                  bool wait)
{
    pulse_ckv_ticks(10 * high_time_us, 10 * low_time_us, wait);
}

bool rmt_busy(void)
{
    return !rmt_tx_done;
}
