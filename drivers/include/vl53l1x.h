#ifndef VL53L1X_H
#define VL53L1X_H

#include "periph/i2c.h"

#define VL53L1X_HW_ADDR_HEX_29                  (0x29)

#define ROI_DEFAULT_SIZE                        (10)


#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VL53L1X_INTERRUPT_POLARITY_LOW = 0,
    VL53L1X_INTERRUPT_POLARITY_HIGH = 1
} vl53l1x_interrupt_polarity_t;

typedef enum {
    VL53L1X_MODE_SHORT = 1,
    VL53L1X_MODE_LONG = 2,
} vl53l1x_distance_mode_t;

typedef enum {
    VL53L1X_TB_15_MS = 15,
    VL53L1X_TB_20_MS = 20,
    VL53L1X_TB_33_MS = 33,
    VL53L1X_TB_50_MS = 50,
    VL53L1X_TB_100_MS =100,
    VL53L1X_TB_200_MS = 200,
    VL53L1X_TB_500_MS = 500  
} vl53l1x_timing_budget_ms_t;

typedef enum {
    VL53L1X_WINDOW_BELOW = 0,
    VL53L1X_WINDOW_ABOVE = 1,
    VL53L1X_WINDOW_OUT = 2,
    VL53L1X_WINDOW_IN = 3,
} vl53l1x_window_mode_t;

typedef struct {
    uint16_t threshold_low;
    uint16_t threshold_high;
    vl53l1x_window_mode_t window;
} vl53l1x_detection_threshold_t;

// typedef struct {
//     uint16_t x,
//     uint16_t y,
//     uint8_t center
// } vl53l1x_region_of_interent_t;

typedef struct {
	uint8_t      major;   
	uint8_t      minor;   
	uint8_t      build;   
	uint32_t     revision;
} vl53l1x_version_t;

typedef struct {
	uint8_t Status;		/*!< ResultStatus */
	uint16_t Distance;	/*!< ResultDistance */
	uint16_t Ambient;	/*!< ResultAmbient */
	uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
	uint16_t NumSPADs;	/*!< ResultNumSPADs */
} vl53l1x_result_t;


/**
 * @brief   Device initialization parameters
 */
typedef struct {
    i2c_t i2c_bus;                  /**< I2C device which is used */
    uint8_t addr;                   /**< Hardware address of the MPU-9X50 */
} vl53l1x_params_t;

/**
 * @brief   Device descriptor for the VL53L1X sensor
 */
typedef struct {
    vl53l1x_params_t params;    /**< Device initialization parameters */
} vl53l1x_t;

// Implementation of vl53l1_platform
int8_t _i2c_hw_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);

int8_t _i2c_hw_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);

int8_t _i2c_hw_WrByte(uint16_t dev, uint16_t index, uint8_t data);

int8_t _i2c_hw_WrWord(uint16_t dev, uint16_t index, uint16_t data);

int8_t _i2c_hw_WrDWord(uint16_t dev, uint16_t index, uint32_t data);

int8_t _i2c_hw_RdByte(uint16_t dev, uint16_t index, uint8_t *data);

int8_t _i2c_hw_RdWord(uint16_t dev, uint16_t index, uint16_t *data);

int8_t _i2c_hw_RdDWord(uint16_t dev, uint16_t index, uint32_t *data);

int8_t _i2c_hw_WaitMs(uint16_t dev, int32_t wait_ms);



int vl53l1x_get_sw_version(vl53l1x_version_t *p_version);

// int vl53l1x_set_i2c_address(vl53l1x_t *dev, uint16_t new_address);

int vl53l1x_init(vl53l1x_t *dev, const vl53l1x_params_t *params);

int vl53l1x_clear_interrupt(vl53l1x_t *dev);

int vl53l1x_set_interrupt_polarity(vl53l1x_t *dev, vl53l1x_interrupt_polarity_t int_pol);

int vl53l1x_get_interrupt_polarity(vl53l1x_t *dev, uint8_t *int_pol) ;

int vl53l1x_start_ranging(vl53l1x_t *dev);

int vl53l1x_stop_ranging(vl53l1x_t *dev);

int vl53l1x_check_for_data_ready(vl53l1x_t *dev, uint8_t *is_data_ready);

int vl53l1x_set_timing_budget(vl53l1x_t *dev, vl53l1x_timing_budget_ms_t timing_budget);

int vl53l1x_get_timing_budget(vl53l1x_t *dev, uint16_t *timing_budget);

int vl53l1x_set_distance_mode(vl53l1x_t *dev, vl53l1x_distance_mode_t distance_mode);

int vl53l1x_get_distance_mode(vl53l1x_t *dev, uint16_t *distance_mode);

int vl53l1x_set_inter_measurements(vl53l1x_t *dev, uint32_t inter_measurement_ms);

int vl53l1x_get_inter_measurements(vl53l1x_t *dev, uint16_t *inter_measurement_ms);

int vl53l1x_get_boot_state(vl53l1x_t *dev, uint8_t *state);

int vl53l1x_get_sensor_id(vl53l1x_t *dev, uint16_t *id);

int vl53l1x_get_distance(vl53l1x_t *dev, uint16_t *distance);

int vl53l1x_get_signal_per_spad(vl53l1x_t *dev, uint16_t *signal_per_sp);

int vl53l1x_get_ambient_per_spad(vl53l1x_t *dev, uint16_t *amb);

int vl53l1x_get_signal_rate(vl53l1x_t *dev, uint16_t *signal_rate);

int vl53l1x_get_span_nb(vl53l1x_t *dev, uint16_t *span_nb);

int vl53l1x_get_ambient_rate(vl53l1x_t *dev, uint16_t *amb_rate);

int vl53l1x_get_range_status(vl53l1x_t *dev, uint8_t *range_status);

int vl53l1x_get_result(vl53l1x_t *dev, vl53l1x_result_t *result);

int vl53l1x_set_offset(vl53l1x_t *dev, int16_t offset_value);

int vl53l1x_get_offset(vl53l1x_t *dev, int16_t *offset);

int vl53l1x_set_xtalk(vl53l1x_t *dev, uint16_t xtalk_value);

int vl53l1x_get_xtalk(vl53l1x_t *dev, uint16_t *xtalk);

int vl53l1x_set_signal_threshold(vl53l1x_t *dev, uint16_t signal);

int vl53l1x_get_signal_threshold(vl53l1x_t *dev, uint16_t *signal);

int vl53l1x_set_sigma_threshold(vl53l1x_t *dev, uint16_t sigma);

int vl53l1x_get_sigma_threshold(vl53l1x_t *dev, uint16_t *sigma);

int vl53l1x_set_distance_threshold(vl53l1x_t *dev, vl53l1x_detection_threshold_t *threshold_mode);

int vl53l1x_get_distance_threashold_window(vl53l1x_t *dev, uint16_t *window);

int vl53l1x_get_distance_threashold_high(vl53l1x_t *dev, uint16_t *threshold_low);

int vl53l1x_get_distance_threashold_low(vl53l1x_t *dev, uint16_t *threshold_high);

int vl53l1x_set_roi(vl53l1x_t *dev, uint16_t x, uint16_t y);

int vl53l1x_get_roi_xy(vl53l1x_t *dev, uint16_t *x, uint16_t *y);

int vl53l1x_set_roi_center(vl53l1x_t *dev, uint8_t roi_center);

int vl53l1x_get_roi_center(vl53l1x_t *dev, uint8_t *roi_center);



int vl53l1x_start_temperature_update(vl53l1x_t *dev);



#ifdef __cplusplus
}
#endif

#endif /* VL53L1X_H */
/** @} */