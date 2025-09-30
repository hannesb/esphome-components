#include "ed097oc4.h"

#include <string.h>

#include "esp_attr.h"         // IRAM_ATTR (ensure attribute is available)
#include "esp_timer.h"
#include "xtensa/core-macros.h"
#include "esp_rom_sys.h"      // esp_rom_delay_us (IDF 5), if you switch to ROM delay later
#include "soc/gpio_reg.h"     // REG_WRITE, GPIO_OUT_W1TS_REG / GPIO_OUT_W1TC_REG

#include "i2s_data_bus.h"
#include "rmt_pulse.h"


typedef struct {
  bool ep_latch_enable : 1;
  bool power_disable : 1;
  bool pos_power_enable : 1;
  bool neg_power_enable : 1;
  bool ep_stv : 1;
  bool ep_scan_direction : 1;
  bool ep_mode : 1;
  bool ep_output_enable : 1;
} epd_config_register_t;

static epd_config_register_t config_reg;

/*
 * Fast GPIO set/clear for IDF 5.x:
 * Use register write macros instead of legacy GPIO.out_w1ts/out_w1tc.
 * (This handles GPIO 0..31; panel wiring uses low GPIOs.)
 */
static inline IRAM_ATTR void fast_gpio_set_hi(gpio_num_t gpio_num) {
  REG_WRITE(GPIO_OUT_W1TS_REG, (1U << (uint32_t)gpio_num));
}

static inline IRAM_ATTR void fast_gpio_set_lo(gpio_num_t gpio_num) {
  REG_WRITE(GPIO_OUT_W1TC_REG, (1U << (uint32_t)gpio_num));
}

void IRAM_ATTR busy_delay(uint32_t cycles) {
  volatile unsigned long counts = XTHAL_GET_CCOUNT() + cycles;
  while (XTHAL_GET_CCOUNT() < counts) {
  };
}

inline static void IRAM_ATTR push_cfg_bit(bool bit) {
  fast_gpio_set_lo(CFG_CLK);
  if (bit) {
    fast_gpio_set_hi(CFG_DATA);
  } else {
    fast_gpio_set_lo(CFG_DATA);
  }
  fast_gpio_set_hi(CFG_CLK);
}

static void IRAM_ATTR push_cfg(epd_config_register_t *cfg) {
  fast_gpio_set_lo(CFG_STR);

  // push config bits in reverse order
  push_cfg_bit(cfg->ep_output_enable);
  push_cfg_bit(cfg->ep_mode);
  push_cfg_bit(cfg->ep_scan_direction);
  push_cfg_bit(cfg->ep_stv);

  push_cfg_bit(cfg->neg_power_enable);
  push_cfg_bit(cfg->pos_power_enable);
  push_cfg_bit(cfg->power_disable);
  push_cfg_bit(cfg->ep_latch_enable);

  fast_gpio_set_hi(CFG_STR);
}

void epd_base_init(uint32_t epd_row_width) {

  config_reg.ep_latch_enable = false;
  config_reg.power_disable = true;
  config_reg.pos_power_enable = false;
  config_reg.neg_power_enable = false;
  config_reg.ep_stv = true;
  config_reg.ep_scan_direction = true;
  config_reg.ep_mode = false;
  config_reg.ep_output_enable = false;

  /* Power Control Output/Off */
  gpio_set_direction(CFG_DATA, GPIO_MODE_OUTPUT);
  gpio_set_direction(CFG_CLK, GPIO_MODE_OUTPUT);
  gpio_set_direction(CFG_STR, GPIO_MODE_OUTPUT);
  fast_gpio_set_lo(CFG_STR);

  push_cfg(&config_reg);

  // Setup I2S
  i2s_bus_config i2s_config;
  // add an offset off dummy bytes to allow for enough timing headroom
  i2s_config.epd_row_width = epd_row_width + 32;
  i2s_config.clock = CKH;
  i2s_config.start_pulse = STH;
  i2s_config.data_0 = D0;
  i2s_config.data_1 = D1;
  i2s_config.data_2 = D2;
  i2s_config.data_3 = D3;
  i2s_config.data_4 = D4;
  i2s_config.data_5 = D5;
  i2s_config.data_6 = D6;
  i2s_config.data_7 = D7;

  i2s_bus_init(&i2s_config);

  rmt_pulse_init(CKV);
}

void epd_poweron() {
  // POWERON
  config_reg.ep_scan_direction = true;
  config_reg.power_disable = false;
  push_cfg(&config_reg);
  busy_delay(100 * 240);
  config_reg.neg_power_enable = true;
  push_cfg(&config_reg);
  busy_delay(500 * 240);
  config_reg.pos_power_enable = true;
  push_cfg(&config_reg);
  busy_delay(100 * 240);
  config_reg.ep_stv = true;
  push_cfg(&config_reg);
  fast_gpio_set_hi(STH);
  // END POWERON
}

void epd_poweroff() {
  // POWEROFF
  config_reg.pos_power_enable = false;
  push_cfg(&config_reg);
  busy_delay(10 * 240);
  config_reg.neg_power_enable = false;
  push_cfg(&config_reg);
  busy_delay(100 * 240);
  config_reg.power_disable = true;
  push_cfg(&config_reg);

  config_reg.ep_stv = false;
  push_cfg(&config_reg);

//   config_reg.ep_scan_direction = false;
//   push_cfg(&config_reg);

  // END POWEROFF
}

void epd_poweroff_all(void) {
  memset(&config_reg, 0, sizeof(config_reg));
  push_cfg(&config_reg);
}

void epd_start_frame(void) {
  while (i2s_is_busy() || rmt_busy()) {
  };
  config_reg.ep_mode = true;
  push_cfg(&config_reg);

  pulse_ckv_us(1, 1, true);

  // This is very timing-sensitive!
  config_reg.ep_stv = false;
  push_cfg(&config_reg);
  busy_delay(240);
  pulse_ckv_us(10, 10, false);
  config_reg.ep_stv = true;
  push_cfg(&config_reg);
  pulse_ckv_us(0, 10, true);

  config_reg.ep_output_enable = true;
  push_cfg(&config_reg);

  pulse_ckv_us(1, 1, true);
}

static inline void latch_row() {
  config_reg.ep_latch_enable = true;
  push_cfg(&config_reg);

  config_reg.ep_latch_enable = false;
  push_cfg(&config_reg);
}

void epd_skip(void) {
#if defined(CONFIG_EPD_DISPLAY_TYPE_ED097TC2)
  pulse_ckv_ticks(2, 2, false);
#else
  // According to the spec, the OC4 maximum CKV frequency is 200kHz.
  pulse_ckv_ticks(45, 5, false);
#endif
}

void epd_output_row(uint32_t output_time_dus) {

  while (i2s_is_busy() || rmt_busy()) {
  };
  latch_row();

  pulse_ckv_ticks(output_time_dus, 50, false);

  i2s_start_line_output();
  i2s_switch_buffer();
}

void epd_end_frame(void) {
  config_reg.ep_output_enable = false;
  push_cfg(&config_reg);
  config_reg.ep_mode = false;
  push_cfg(&config_reg);
  pulse_ckv_us(1, 1, true);
  pulse_ckv_us(1, 1, true);
}

void epd_switch_buffer(void) { i2s_switch_buffer(); }
uint8_t *epd_get_current_buffer(void) {
  return (uint8_t *) i2s_get_current_buffer();
}
