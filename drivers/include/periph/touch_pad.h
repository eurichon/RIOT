#ifndef PERIPH_TOUCH_PAD_H
#define PERIPH_TOUCH_PAD_H

#include "touch_pad_types.h"


typedef struct {
    touch_high_volt_t refh;
    touch_low_volt_t refl;
    touch_volt_atten_t atten;
} touch_hal_volt_t;

void touch_pad_init(void);

void touch_pad_deinit(void);

void touch_pad_set_voltage(const touch_hal_volt_t *volt);

int touch_pad_io_init(touch_pad_t touch_num);

void touch_pad_config(touch_pad_t touch_num, uint16_t threshold);

void touch_pad_get_fsm_mode(touch_fsm_mode_t *mode);

int touch_pad_read(touch_pad_t touch_num, uint16_t *touch_value, touch_fsm_mode_t mode);

void touch_pad_read_raw_data(touch_pad_t touch_num, uint16_t *touch_value);

#endif // PERIPH_TOUCH_PAD_H