#ifndef PERIPH_TOUCH_PAD_H
#define PERIPH_TOUCH_PAD_H

#include "touch_pad_types.h"

// #include "esp_intr.h"
// #include "esp_err.h"
// #include "esp_intr_alloc.h"
// #include "soc/touch_channel.h"

typedef void (*touch_pad_intr_handler_t)(uint8_t pin, void *arg);
typedef void (*touch_pad_intr_arg_t);

typedef struct {
    touch_high_volt_t refh;
    touch_low_volt_t refl;
    touch_volt_atten_t atten;
} touch_hal_volt_t;

void touch_pad_init(void);

void touch_pad_deinit(void);

void touch_pad_set_voltage( touch_high_volt_t refh, touch_low_volt_t refl, touch_volt_atten_t atten);

int touch_pad_io_init(touch_pad_t touch_num);

void touch_pad_config(touch_pad_t touch_num, uint16_t threshold);

void touch_pad_get_fsm_mode(touch_fsm_mode_t *mode);

int touch_pad_read(touch_pad_t touch_num, uint16_t *touch_value);

void touch_pad_read_raw_data(touch_pad_t touch_num, uint16_t *touch_value);

// for interrupt trigger

void touch_pad_start_fsm(void);

void touch_pad_stop_fsm(void);

void touch_pad_set_fsm_mode(touch_fsm_mode_t mode);

void touch_pad_get_threshold(touch_pad_t touch_num, uint16_t *threshold);

uint16_t touch_pad_get_status(void);

uint8_t touch_pad_status_to_pin(uint16_t status);

void touch_pad_clear_status(void);

void touch_pad_intr_enable(void);

void touch_pad_set_thresh(touch_pad_t touch_num, uint16_t threshold);

void touch_pad_intr_disable(void);

int touch_pad_isr_register(touch_pad_intr_handler_t cb, void *arg);

#endif // PERIPH_TOUCH_PAD_H