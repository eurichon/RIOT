#include <stdio.h> // for debugging
#include <stdlib.h>

#include "board.h"
#include "periph/touch_pad.h"

#include "soc/sens_struct.h"
#include "soc/rtc_cntl_struct.h"
#include "soc/rtc_io_struct.h"
#include "soc/rtc_io_reg.h"
#include "soc/dport_reg.h"

#include "esp_attr.h"
#include "esp/common_macros.h"
#include "esp_common.h"

#include "rom/ets_sys.h"
#include "xtensa/xtensa_api.h"


#include "irq_arch.h"
#include "soc/rtc_periph.h"
#include "esp_intr_alloc.h"
// #include "esp_sleep.h"


#include "gpio_arch.h"
#include "periph/gpio.h"

#define RTC_GPIO_MODE_DISABLED              (3U)


// /* Store IO number corresponding to the Touch Sensor channel number. */
const int touch_sensor_channel_io_map[SOC_TOUCH_SENSOR_NUM] = {
    TOUCH_PAD_NUM0_GPIO_NUM,
    TOUCH_PAD_NUM1_GPIO_NUM,
    TOUCH_PAD_NUM2_GPIO_NUM,
    TOUCH_PAD_NUM3_GPIO_NUM,
    TOUCH_PAD_NUM4_GPIO_NUM,
    TOUCH_PAD_NUM5_GPIO_NUM,
    TOUCH_PAD_NUM6_GPIO_NUM,
    TOUCH_PAD_NUM7_GPIO_NUM,
    TOUCH_PAD_NUM8_GPIO_NUM,
    TOUCH_PAD_NUM9_GPIO_NUM
};

#define TOUCH_GET_IO_NUM(channel) (touch_sensor_channel_io_map[channel])


const rtc_io_desc_t rtc_io_desc[SOC_RTCIO_PIN_COUNT] = {
    /*REG                    MUX select                  function select              Input enable                Pullup                   Pulldown                 Sleep select                 Sleep input enable             PAD hold                  Mask of drive capability Offset                   gpio number */
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE1_MUX_SEL_M,    RTC_IO_SENSE1_FUN_SEL_S,     RTC_IO_SENSE1_FUN_IE_M,     0,                       0,                       RTC_IO_SENSE1_SLP_SEL_M,     RTC_IO_SENSE1_SLP_IE_M,     0, RTC_IO_SENSE1_HOLD_M,     0,                       0,                       RTCIO_CHANNEL_0_GPIO_NUM}, //36
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE2_MUX_SEL_M,    RTC_IO_SENSE2_FUN_SEL_S,     RTC_IO_SENSE2_FUN_IE_M,     0,                       0,                       RTC_IO_SENSE2_SLP_SEL_M,     RTC_IO_SENSE2_SLP_IE_M,     0, RTC_IO_SENSE2_HOLD_M,     0,                       0,                       RTCIO_CHANNEL_1_GPIO_NUM}, //37
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE3_MUX_SEL_M,    RTC_IO_SENSE3_FUN_SEL_S,     RTC_IO_SENSE3_FUN_IE_M,     0,                       0,                       RTC_IO_SENSE3_SLP_SEL_M,     RTC_IO_SENSE3_SLP_IE_M,     0, RTC_IO_SENSE3_HOLD_M,     0,                       0,                       RTCIO_CHANNEL_2_GPIO_NUM}, //38
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE4_MUX_SEL_M,    RTC_IO_SENSE4_FUN_SEL_S,     RTC_IO_SENSE4_FUN_IE_M,     0,                       0,                       RTC_IO_SENSE4_SLP_SEL_M,     RTC_IO_SENSE4_SLP_IE_M,     0, RTC_IO_SENSE4_HOLD_M,     0,                       0,                       RTCIO_CHANNEL_3_GPIO_NUM}, //39
    {RTC_IO_ADC_PAD_REG,     RTC_IO_ADC1_MUX_SEL_M,      RTC_IO_ADC1_FUN_SEL_S,       RTC_IO_ADC1_FUN_IE_M,       0,                       0,                       RTC_IO_ADC1_SLP_SEL_M,       RTC_IO_ADC1_SLP_IE_M,       0, RTC_IO_ADC1_HOLD_M,       0,                       0,                       RTCIO_CHANNEL_4_GPIO_NUM}, //34
    {RTC_IO_ADC_PAD_REG,     RTC_IO_ADC2_MUX_SEL_M,      RTC_IO_ADC2_FUN_SEL_S,       RTC_IO_ADC2_FUN_IE_M,       0,                       0,                       RTC_IO_ADC2_SLP_SEL_M,       RTC_IO_ADC2_SLP_IE_M,       0, RTC_IO_ADC2_HOLD_M,       0,                       0,                       RTCIO_CHANNEL_5_GPIO_NUM}, //35
    {RTC_IO_PAD_DAC1_REG,    RTC_IO_PDAC1_MUX_SEL_M,     RTC_IO_PDAC1_FUN_SEL_S,      RTC_IO_PDAC1_FUN_IE_M,      RTC_IO_PDAC1_RUE_M,      RTC_IO_PDAC1_RDE_M,      RTC_IO_PDAC1_SLP_SEL_M,      RTC_IO_PDAC1_SLP_IE_M,      0, RTC_IO_PDAC1_HOLD_M,      RTC_IO_PDAC1_DRV_V,      RTC_IO_PDAC1_DRV_S,      RTCIO_CHANNEL_6_GPIO_NUM}, //25
    {RTC_IO_PAD_DAC2_REG,    RTC_IO_PDAC2_MUX_SEL_M,     RTC_IO_PDAC2_FUN_SEL_S,      RTC_IO_PDAC2_FUN_IE_M,      RTC_IO_PDAC2_RUE_M,      RTC_IO_PDAC2_RDE_M,      RTC_IO_PDAC2_SLP_SEL_M,      RTC_IO_PDAC2_SLP_IE_M,      0, RTC_IO_PDAC2_HOLD_M,      RTC_IO_PDAC2_DRV_V,      RTC_IO_PDAC2_DRV_S,      RTCIO_CHANNEL_7_GPIO_NUM}, //26
    {RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32N_MUX_SEL_M,     RTC_IO_X32N_FUN_SEL_S,       RTC_IO_X32N_FUN_IE_M,       RTC_IO_X32N_RUE_M,       RTC_IO_X32N_RDE_M,       RTC_IO_X32N_SLP_SEL_M,       RTC_IO_X32N_SLP_IE_M,       0, RTC_IO_X32N_HOLD_M,       RTC_IO_X32N_DRV_V,       RTC_IO_X32N_DRV_S,       RTCIO_CHANNEL_8_GPIO_NUM}, //33
    {RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32P_MUX_SEL_M,     RTC_IO_X32P_FUN_SEL_S,       RTC_IO_X32P_FUN_IE_M,       RTC_IO_X32P_RUE_M,       RTC_IO_X32P_RDE_M,       RTC_IO_X32P_SLP_SEL_M,       RTC_IO_X32P_SLP_IE_M,       0, RTC_IO_X32P_HOLD_M,       RTC_IO_X32P_DRV_V,       RTC_IO_X32P_DRV_S,       RTCIO_CHANNEL_9_GPIO_NUM}, //32
    {RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_MUX_SEL_M, RTC_IO_TOUCH_PAD0_FUN_SEL_S, RTC_IO_TOUCH_PAD0_FUN_IE_M, RTC_IO_TOUCH_PAD0_RUE_M, RTC_IO_TOUCH_PAD0_RDE_M, RTC_IO_TOUCH_PAD0_SLP_SEL_M, RTC_IO_TOUCH_PAD0_SLP_IE_M, 0, RTC_IO_TOUCH_PAD0_HOLD_M, RTC_IO_TOUCH_PAD0_DRV_V, RTC_IO_TOUCH_PAD0_DRV_S, RTCIO_CHANNEL_10_GPIO_NUM},// 4
    {RTC_IO_TOUCH_PAD1_REG, RTC_IO_TOUCH_PAD1_MUX_SEL_M, RTC_IO_TOUCH_PAD1_FUN_SEL_S, RTC_IO_TOUCH_PAD1_FUN_IE_M, RTC_IO_TOUCH_PAD1_RUE_M, RTC_IO_TOUCH_PAD1_RDE_M, RTC_IO_TOUCH_PAD1_SLP_SEL_M, RTC_IO_TOUCH_PAD1_SLP_IE_M, 0, RTC_IO_TOUCH_PAD1_HOLD_M, RTC_IO_TOUCH_PAD1_DRV_V, RTC_IO_TOUCH_PAD1_DRV_S, RTCIO_CHANNEL_11_GPIO_NUM},// 0
    {RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_MUX_SEL_M, RTC_IO_TOUCH_PAD2_FUN_SEL_S, RTC_IO_TOUCH_PAD2_FUN_IE_M, RTC_IO_TOUCH_PAD2_RUE_M, RTC_IO_TOUCH_PAD2_RDE_M, RTC_IO_TOUCH_PAD2_SLP_SEL_M, RTC_IO_TOUCH_PAD2_SLP_IE_M, 0, RTC_IO_TOUCH_PAD2_HOLD_M, RTC_IO_TOUCH_PAD2_DRV_V, RTC_IO_TOUCH_PAD2_DRV_S, RTCIO_CHANNEL_12_GPIO_NUM},// 2
    {RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_MUX_SEL_M, RTC_IO_TOUCH_PAD3_FUN_SEL_S, RTC_IO_TOUCH_PAD3_FUN_IE_M, RTC_IO_TOUCH_PAD3_RUE_M, RTC_IO_TOUCH_PAD3_RDE_M, RTC_IO_TOUCH_PAD3_SLP_SEL_M, RTC_IO_TOUCH_PAD3_SLP_IE_M, 0, RTC_IO_TOUCH_PAD3_HOLD_M, RTC_IO_TOUCH_PAD3_DRV_V, RTC_IO_TOUCH_PAD3_DRV_S, RTCIO_CHANNEL_13_GPIO_NUM},//15
    {RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_MUX_SEL_M, RTC_IO_TOUCH_PAD4_FUN_SEL_S, RTC_IO_TOUCH_PAD4_FUN_IE_M, RTC_IO_TOUCH_PAD4_RUE_M, RTC_IO_TOUCH_PAD4_RDE_M, RTC_IO_TOUCH_PAD4_SLP_SEL_M, RTC_IO_TOUCH_PAD4_SLP_IE_M, 0, RTC_IO_TOUCH_PAD4_HOLD_M, RTC_IO_TOUCH_PAD4_DRV_V, RTC_IO_TOUCH_PAD4_DRV_S, RTCIO_CHANNEL_14_GPIO_NUM},//13
    {RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_MUX_SEL_M, RTC_IO_TOUCH_PAD5_FUN_SEL_S, RTC_IO_TOUCH_PAD5_FUN_IE_M, RTC_IO_TOUCH_PAD5_RUE_M, RTC_IO_TOUCH_PAD5_RDE_M, RTC_IO_TOUCH_PAD5_SLP_SEL_M, RTC_IO_TOUCH_PAD5_SLP_IE_M, 0, RTC_IO_TOUCH_PAD5_HOLD_M, RTC_IO_TOUCH_PAD5_DRV_V, RTC_IO_TOUCH_PAD5_DRV_S, RTCIO_CHANNEL_15_GPIO_NUM},//12
    {RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_MUX_SEL_M, RTC_IO_TOUCH_PAD6_FUN_SEL_S, RTC_IO_TOUCH_PAD6_FUN_IE_M, RTC_IO_TOUCH_PAD6_RUE_M, RTC_IO_TOUCH_PAD6_RDE_M, RTC_IO_TOUCH_PAD6_SLP_SEL_M, RTC_IO_TOUCH_PAD6_SLP_IE_M, 0, RTC_IO_TOUCH_PAD6_HOLD_M, RTC_IO_TOUCH_PAD6_DRV_V, RTC_IO_TOUCH_PAD6_DRV_S, RTCIO_CHANNEL_16_GPIO_NUM},//14
    {RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_MUX_SEL_M, RTC_IO_TOUCH_PAD7_FUN_SEL_S, RTC_IO_TOUCH_PAD7_FUN_IE_M, RTC_IO_TOUCH_PAD7_RUE_M, RTC_IO_TOUCH_PAD7_RDE_M, RTC_IO_TOUCH_PAD7_SLP_SEL_M, RTC_IO_TOUCH_PAD7_SLP_IE_M, 0, RTC_IO_TOUCH_PAD7_HOLD_M, RTC_IO_TOUCH_PAD7_DRV_V, RTC_IO_TOUCH_PAD7_DRV_S, RTCIO_CHANNEL_17_GPIO_NUM},//27
};


// static uint16_t s_touch_pad_init_bit = 0x0000;

/* 
 * checks if a gpio pin is given and if it is unused
 */
static inline bool rtc_gpio_is_valid_gpio(gpio_t gpio_num) {
    return (gpio_num < SOC_GPIO_PIN_COUNT && gpio_is_rtcio(gpio_num) >= 0);
}

/* 
 * convert the gpio number to an rtcio number
 */
static inline int rtc_io_number_get(gpio_t gpio_num) {
    return gpio_is_rtcio(gpio_num);
}


/**
 * Swap the number of touch8 and touch9.
 *
 * @touch_num Touch channel num.
 */
static inline touch_pad_t touch_corrert_nums(touch_pad_t touch_num)
{
    if (touch_num == TOUCH_PAD_NUM8) {
        return TOUCH_PAD_NUM9;
    } else if (touch_num == TOUCH_PAD_NUM9) {
        return TOUCH_PAD_NUM8;
    }
    return touch_num;
}


/**
 * Get touch sensor measure status. No block.
 *
 * @return
 *      - If touch sensors measure done.
 */
static inline bool touch_ll_meas_is_done(void)
{
    return (bool)SENS.sar_touch_ctrl2.touch_meas_done;
}

/**
 * Get touch sensor raw data (touch sensor counter value) from register. No block.
 *
 * @param touch_num touch pad index.
 * @return touch_value pointer to accept touch sensor value.
 */
static inline uint32_t touch_ll_read_raw_data(touch_pad_t touch_num)
{
    touch_pad_t tp_wrap = touch_corrert_nums(touch_num);
    return (tp_wrap & 0x1) ? (SENS.touch_meas[tp_wrap / 2].l_val) : (SENS.touch_meas[tp_wrap / 2].h_val);
}


void _touch_pad_sw_start(void);



void touch_pad_init(void) {
    // puts("Hello from touch init!");

    // stops the FSM timer
    RTCCNTL.state0.touch_slp_timer_en = 0;

    // disables touch pad interrupt
    RTCCNTL.int_ena.rtc_touch = 0;  

    // clears touch pad interrupt            
    RTCCNTL.int_clr.rtc_touch = 1;              

    // disable touch sensor channel by bitmask
    SENS.sar_touch_enable.touch_pad_worken &= TOUCH_LL_BITS_SWAP(~TOUCH_PAD_BIT_MASK_ALL); 

    // clear touch sensor group mask
    SENS.sar_touch_enable.touch_pad_outen1 &= TOUCH_LL_BITS_SWAP(~TOUCH_PAD_BIT_MASK_ALL);
    SENS.sar_touch_enable.touch_pad_outen2 &= TOUCH_LL_BITS_SWAP(~TOUCH_PAD_BIT_MASK_ALL);

    // set touch sensor interrupt trigger mode
    SENS.sar_touch_ctrl1.touch_out_sel = TOUCH_TRIGGER_MODE_DEFAULT;

    // sets touch sensor interrupt trigger source
    SENS.sar_touch_ctrl1.touch_out_1en = TOUCH_TRIGGER_SOURCE_DEFAULT;

    // clear all touch sensor status
    SENS.sar_touch_ctrl2.touch_meas_en_clr = 1;

    //Set touch sensor measurement time
    SENS.sar_touch_ctrl1.touch_meas_delay = TOUCH_PAD_MEASURE_CYCLE_DEFAULT;        // here maybe HAL_FORCE_MODIFY_U32_REG_FIELD
    SENS.sar_touch_ctrl1.touch_xpd_wait = SOC_TOUCH_PAD_MEASURE_WAIT_MAX;           // here maybe HAL_FORCE_MODIFY_U32_REG_FIELD

    // set touch sensor FSM mode
    SENS.sar_touch_ctrl2.touch_start_fsm_en = 1;
    SENS.sar_touch_ctrl2.touch_start_en = 0;
    SENS.sar_touch_ctrl2.touch_start_force = TOUCH_FSM_MODE_DEFAULT;

    // start touch sensor FSM timer
    RTCCNTL.state0.touch_slp_timer_en = 1;
}


void touch_pad_deinit(void) {
    // stops the FSM timer
    RTCCNTL.state0.touch_slp_timer_en = 0;

    // clear all touch sensor status
    SENS.sar_touch_ctrl2.touch_meas_en_clr = 1;

    // disables touch pad interrupt
    RTCCNTL.int_ena.rtc_touch = 0;  
}


void touch_pad_set_voltage( touch_high_volt_t refh, touch_low_volt_t refl, touch_volt_atten_t atten) {

    // set high voltage
    RTCIO.touch_cfg.drefh = refh;

    // set low voltage
    RTCIO.touch_cfg.drefl = refl;
    
    // set attenuation
    RTCIO.touch_cfg.drange = atten;
}

/* 
 * sets threshold of the pads to the given value         
 */
void touch_sensor_set_threshold(touch_pad_t touch_num, uint16_t threshold) {  
    touch_pad_t tp_wrap = touch_corrert_nums(touch_num); 

    // printf("Tp wrap is: %i\n", tp_wrap);

    if (tp_wrap & 0x1) {                                            
        uint32_t temp_val = SENS.touch_thresh[tp_wrap / 2].val;
        sens_dev_t temp_reg;
        temp_reg.touch_thresh[tp_wrap / 2].val = temp_val;
        temp_reg.touch_thresh[tp_wrap / 2].l_thresh = threshold;
        SENS.touch_thresh[tp_wrap / 2].val = temp_reg.touch_thresh[tp_wrap / 2].val;

        // SENS.touch_thresh[tp_wrap / 2].l_thresh = threshold;
    } else {
        uint32_t temp_val = SENS.touch_thresh[tp_wrap / 2].val;
        sens_dev_t temp_reg;
        temp_reg.touch_thresh[tp_wrap / 2].val = temp_val;
        temp_reg.touch_thresh[tp_wrap / 2].h_thresh = threshold;
        SENS.touch_thresh[tp_wrap / 2].val = temp_reg.touch_thresh[tp_wrap / 2].val;

        // SENS.touch_thresh[tp_wrap / 2].h_thresh = threshold;
    }
}


void touch_pad_get_threshold(touch_pad_t touch_num, uint16_t *threshold) {  
    touch_pad_t tp_wrap = touch_corrert_nums(touch_num); 

    // printf("Tp wrap is: %i\n", tp_wrap);

    if (tp_wrap & 0x1) {    
        *threshold = SENS.touch_thresh[tp_wrap / 2].l_thresh;                                      
    } else {
        *threshold = SENS.touch_thresh[tp_wrap / 2].h_thresh;     
    }
}

/* 
 * touch hal config
 */
void touch_sensor_config(touch_pad_t touch_num) {
    touch_sensor_set_threshold(touch_num, TOUCH_PAD_THRESHOLD_MAX);
    
    // set slope
    RTCIO.touch_pad[touch_num].dac = TOUCH_PAD_SLOPE_DEFAULT;       
    
    // set tie option
    touch_pad_t tp_wrap = touch_corrert_nums(touch_num); 
    RTCIO.touch_pad[tp_wrap].tie_opt = TOUCH_PAD_TIE_OPT_DEFAULT;   
}


int touch_pad_clear_group_mask(uint16_t set1_mask, uint16_t set2_mask, uint16_t en_mask) {
    SENS.sar_touch_enable.touch_pad_worken &= TOUCH_LL_BITS_SWAP(~en_mask);

    SENS.sar_touch_enable.touch_pad_outen1 &= TOUCH_LL_BITS_SWAP(~set1_mask);
    SENS.sar_touch_enable.touch_pad_outen2 &= TOUCH_LL_BITS_SWAP(~set2_mask);

    return 0;
}

int touch_pad_set_group_mask(uint16_t set1_mask, uint16_t set2_mask, uint16_t en_mask) {
    SENS.sar_touch_enable.touch_pad_worken |= TOUCH_LL_BITS_SWAP(en_mask);

    SENS.sar_touch_enable.touch_pad_outen1 |= TOUCH_LL_BITS_SWAP(set1_mask);
    SENS.sar_touch_enable.touch_pad_outen2 |= TOUCH_LL_BITS_SWAP(set2_mask);

    return 0;
}


void touch_pad_config(touch_pad_t touch_num, uint16_t threshold) {
    touch_fsm_mode_t mode;

    if(touch_pad_io_init(touch_num) != 0) {
        puts("Error in configuring io");
    }

    touch_sensor_config(touch_num);    

    touch_sensor_set_threshold(touch_num, threshold);

    touch_pad_get_fsm_mode(&mode);

    if (mode == TOUCH_FSM_MODE_SW) {
        touch_pad_clear_group_mask((1 << touch_num), (1 << touch_num), (1 << touch_num));
    } else if(mode == TOUCH_FSM_MODE_TIMER) {

        // puts("Trigger timer mode");

        touch_pad_set_group_mask((1 << touch_num), (1 << touch_num), (1 << touch_num));
    }
}

void touch_pad_get_fsm_mode(touch_fsm_mode_t *mode) {
    *mode = (touch_fsm_mode_t)SENS.sar_touch_ctrl2.touch_start_force;
}

int touch_pad_io_init(touch_pad_t touch_num) {
    // TOUCH_CHANNEL_CHECK(touch_num);
    gpio_t gpio_num = TOUCH_GET_IO_NUM(touch_num);

    // check if pin is valid
    if (!rtc_gpio_is_valid_gpio(gpio_num)) {
        return -1;
    }

    // convert gpio number to a rtcio number
    int rtcio_num = rtc_io_number_get(gpio_num);

    // connect gpio to analog RTC module
    SET_PERI_REG_MASK(rtc_io_desc[rtcio_num].reg, (rtc_io_desc[rtcio_num].mux));
    // 0:rtc function 1,2,3: reserved
    SET_PERI_REG_BITS(rtc_io_desc[rtcio_num].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, RTCIO_LL_PIN_FUNC, rtc_io_desc[rtcio_num].func);

    // set the direction
    uint8_t rtc_gpio_mode_disabled = (uint8_t) RTC_GPIO_MODE_DISABLED;
    RTCIO.pin[rtcio_num].pad_driver = rtc_gpio_mode_disabled;

    // disable pullup register
    if (rtc_io_desc[rtcio_num].pullup) {
        CLEAR_PERI_REG_MASK(rtc_io_desc[rtcio_num].reg, rtc_io_desc[rtcio_num].pullup);
    }

    // disable pulldown registor    
    if (rtc_io_desc[rtcio_num].pulldown) {
        CLEAR_PERI_REG_MASK(rtc_io_desc[rtcio_num].reg, rtc_io_desc[rtcio_num].pulldown);
    }

    return 0;
}


void _touch_pad_sw_start(void) {
    SENS.sar_touch_ctrl2.touch_start_en = 0;
    SENS.sar_touch_ctrl2.touch_start_en = 1;
}


int touch_pad_read(touch_pad_t touch_num, uint16_t *touch_value) {
    touch_fsm_mode_t mode;
    touch_pad_get_fsm_mode(&mode);

    if (mode == TOUCH_FSM_MODE_SW) {
        touch_pad_set_group_mask((1 << touch_num), (1 << touch_num), (1 << touch_num));

        _touch_pad_sw_start();

        while (!touch_ll_meas_is_done()) {};
        *touch_value = touch_ll_read_raw_data(touch_num);

        touch_pad_clear_group_mask((1 << touch_num), (1 << touch_num), (1 << touch_num));

        return 0;
    } else if (mode == TOUCH_FSM_MODE_TIMER) {
        while (!touch_ll_meas_is_done()) {};
        *touch_value = touch_ll_read_raw_data(touch_num);

        return 0;
    }

    return -1;
}


// for interrupt trigger source

void touch_pad_start_fsm(void) {
    RTCCNTL.state0.touch_slp_timer_en = 1;
}

void touch_pad_stop_fsm(void) {
    RTCCNTL.state0.touch_slp_timer_en = 0;
}

void touch_pad_set_fsm_mode(touch_fsm_mode_t mode) {
    SENS.sar_touch_ctrl2.touch_start_fsm_en = 1;
    SENS.sar_touch_ctrl2.touch_start_en = 0;
    SENS.sar_touch_ctrl2.touch_start_force = mode;

    if (mode == TOUCH_FSM_MODE_TIMER) {
        touch_pad_start_fsm();
    } else {
        touch_pad_stop_fsm();
    }
}

uint16_t touch_pad_get_status(void) {
    return TOUCH_LL_BITS_SWAP(SENS.sar_touch_ctrl2.touch_meas_en);
}

uint8_t touch_pad_status_to_pin(uint16_t status) {
    switch (status) {
        case 1:
            return 0;
        case 2:
            return 1;
        case 4:
            return 2;
        case 8:
            return 3;
        case 16:
            return 4;
        case 32:
            return 5;
        case 64:
            return 6;
        case 128:
            return 7;
        case 256:
            return 8;
        case 512:
            return 9;
        default:
            return 255;
    }
}

void touch_pad_clear_status(void) {
    SENS.sar_touch_ctrl2.touch_meas_en_clr = 1;
}

void touch_pad_intr_enable(void) {
    RTCCNTL.int_ena.rtc_touch = 1;
}

void touch_pad_intr_disable(void) {
    RTCCNTL.int_ena.rtc_touch = 0;
}

void touch_pad_set_thresh(touch_pad_t touch_num, uint16_t threshold) {
    touch_sensor_set_threshold(touch_num, threshold);
}

void touch_pad_haha(void) {
    RTCCNTL.int_ena.rtc_touch = 0;
    RTCCNTL.int_clr.rtc_touch = 1;
}

typedef struct {
    touch_pad_intr_handler_t cb;
    touch_pad_intr_arg_t arg;
} touch_pad_cb_wrapper_arg_t;

touch_pad_cb_wrapper_arg_t wrappers[SOC_TOUCH_SENSOR_NUM];
static int touch_pad_wrapper_counter = -1;

static void IRAM touch_pad_cb_wrapper(void *arg) {
    RTCCNTL.int_ena.rtc_touch = 0;
    RTCCNTL.int_clr.rtc_touch = 1;

    touch_pad_cb_wrapper_arg_t *wrapper = (touch_pad_cb_wrapper_arg_t *)arg;

    uint16_t status = touch_pad_get_status();
    uint8_t touch_pin = touch_pad_status_to_pin(status);

    if (touch_pin != 255) {
        wrapper->cb(touch_pin ,wrapper->arg);
    }

    SENS.sar_touch_ctrl2.touch_meas_en_clr = 1; 
    RTCCNTL.int_ena.rtc_touch = 1;
}

int touch_pad_isr_register(touch_pad_intr_handler_t cb, void *arg) {    
    if (touch_pad_wrapper_counter >= SOC_TOUCH_SENSOR_NUM) {
        return -1;
    }
    
    REG_WRITE(RTC_CNTL_INT_ENA_REG, 0);
    REG_WRITE(RTC_CNTL_INT_CLR_REG, UINT32_MAX);

    // int error = esp_intr_alloc(ETS_RTC_CORE_INTR_SOURCE, 0, yolo, NULL, NULL);

    /* route all interrupt sources to the same RTT level type interrupt */
    intr_matrix_set(PRO_CPU_NUM, ETS_RTC_CORE_INTR_SOURCE, CPU_INUM_RTC);

    /* set interrupt handler and enable the CPU interrupt */
    touch_pad_wrapper_counter++;
    wrappers[touch_pad_wrapper_counter].cb = cb;
    wrappers[touch_pad_wrapper_counter].arg = arg;

    xt_set_interrupt_handler(CPU_INUM_RTC, 
                             &touch_pad_cb_wrapper, 
                             (void *)&wrappers[touch_pad_wrapper_counter]);

    // xt_set_interrupt_handler(CPU_INUM_RTC, &yolo, NULL);

    xt_ints_on(BIT(CPU_INUM_RTC));
    // printf("Error is: %i\n", error);
    
    return 0;
}
