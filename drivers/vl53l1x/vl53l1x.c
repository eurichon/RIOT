#include "vl53l1x.h"
#include "VL53L1X_api.h"

#include "periph/i2c.h"
#include "xtimer.h"
#include "assert.h"

#define I2C_DEVICE                  (0)
#define INT_ON_NO_TARGET            (0)



int8_t _i2c_hw_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	i2c_acquire(I2C_DEVICE);
	int status = i2c_write_regs(I2C_DEVICE, dev, index, pdata, count, I2C_REG16);
	i2c_release(I2C_DEVICE);

	return status;
}

int8_t _i2c_hw_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
	i2c_acquire(I2C_DEVICE);
	int status = i2c_read_regs(I2C_DEVICE, dev, index, pdata, count, I2C_REG16);
	i2c_release(I2C_DEVICE);

	return status;
}

int8_t _i2c_hw_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
	i2c_acquire(I2C_DEVICE);
	int status = i2c_write_reg(I2C_DEVICE, dev, index, data, I2C_REG16);
	i2c_release(I2C_DEVICE);

	return status;
}

int8_t _i2c_hw_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
	uint8_t buff[2];
	buff[1] = data & 0xFF;
	buff[0] = data >> 8;

	int status = VL53L1_WriteMulti(dev, index, buff, 2);

	return status;
}

int8_t _i2c_hw_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
	uint8_t buff[4];
	buff[3] = data & 0xFF;
	buff[2] = data >> 8;
	buff[1] = data >> 16;
	buff[0] = data >> 24;

	int status = VL53L1_WriteMulti(dev, index, buff, 4);

	return status;
}

int8_t _i2c_hw_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
	i2c_acquire(I2C_DEVICE);
	int status = i2c_read_reg(I2C_DEVICE, dev, index, data, I2C_REG16);
	i2c_release(I2C_DEVICE);

	return status;
}

int8_t _i2c_hw_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	uint8_t buff[2];
	
	int status = VL53L1_ReadMulti(dev, index, buff, 2);

	uint16_t tmp;
	tmp = buff[0];
	tmp <<= 8;
	tmp |= buff[1];
	*data = tmp;

	return status;
}

int8_t _i2c_hw_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
	uint8_t buff[4];

	int status = VL53L1_ReadMulti(dev, index, buff, 4);

	uint32_t tmp;
	tmp = buff[0];
	tmp <<= 8;
	tmp |= buff[1];
	tmp <<= 8;
	tmp |= buff[2];
	tmp <<= 8;
	tmp |= buff[3];

	*data = tmp;

	return status;
}

int8_t _i2c_hw_WaitMs(uint16_t dev, int32_t wait_ms){
	xtimer_msleep(wait_ms);

	return 0;
}

// wrapper==================================================


int vl53l1x_get_sw_version(vl53l1x_version_t *p_version) {
    VL53L1X_GetSWVersion((VL53L1X_Version_t *)p_version);

    return 0;
}

int vl53l1x_set_i2c_address(vl53l1x_t *dev, uint16_t new_address) {
    uint8_t status = VL53L1X_SetI2CAddress(dev->params.addr, new_address);

    if (status == 0) {
        dev->params.addr = new_address;
        return 0;
    } else {
        return -1;
    }
}

int vl53l1x_init(vl53l1x_t *dev, const vl53l1x_params_t *params) {
    dev->params = *params;

    VL53L1X_SensorInit(dev->params.addr);
    VL53L1X_SetDistanceMode(dev->params.addr, VL53L1X_MODE_LONG);
    VL53L1X_SetTimingBudgetInMs(dev->params.addr, VL53L1X_TB_100_MS);
    VL53L1X_SetInterMeasurementInMs(dev->params.addr, VL53L1X_TB_100_MS);
    VL53L1X_StartRanging(dev->params.addr);

    return 0;
}

int vl53l1x_clear_interrupt(vl53l1x_t *dev) {
    VL53L1X_ClearInterrupt(dev->params.addr);

    return 0; 
}

int vl53l1x_set_interrupt_polarity(vl53l1x_t *dev, vl53l1x_interrupt_polarity_t int_pol) {
    VL53L1X_SetInterruptPolarity(dev->params.addr, int_pol);

    return 0;
}

int vl53l1x_get_interrupt_polarity(vl53l1x_t *dev, uint8_t *int_pol) {
    VL53L1X_GetInterruptPolarity(dev->params.addr, int_pol);

    return 0;   
}


int vl53l1x_start_ranging(vl53l1x_t *dev) {
    VL53L1X_StartRanging(dev->params.addr);

    return 0;
}

int vl53l1x_stop_ranging(vl53l1x_t *dev) {
    VL53L1X_StopRanging(dev->params.addr);

    return 0;
}

int vl53l1x_check_for_data_ready(vl53l1x_t *dev, uint8_t *is_data_ready) {
    VL53L1X_CheckForDataReady(dev->params.addr, is_data_ready);

    return 0;
}

int vl53l1x_set_timing_budget(vl53l1x_t *dev, vl53l1x_timing_budget_ms_t timing_budget) {
    switch (timing_budget) {
        case VL53L1X_TB_15_MS:
        case VL53L1X_TB_20_MS:
        case VL53L1X_TB_33_MS:
        case VL53L1X_TB_50_MS:
        case VL53L1X_TB_100_MS:
        case VL53L1X_TB_200_MS:
        case VL53L1X_TB_500_MS:
            VL53L1X_SetTimingBudgetInMs(dev->params.addr, timing_budget);
            break;
        default:
            return -1;
    }

    return 0;
}

int vl53l1x_get_timing_budget(vl53l1x_t *dev, uint16_t *timing_budget) {
    VL53L1X_GetTimingBudgetInMs(dev->params.addr,  timing_budget);

    return 0;
}

int vl53l1x_set_distance_mode(vl53l1x_t *dev, vl53l1x_distance_mode_t distance_mode) {
    if (distance_mode == VL53L1X_MODE_SHORT || distance_mode == VL53L1X_MODE_LONG) {
        VL53L1X_SetDistanceMode(dev->params.addr, distance_mode);
    } else {
        return -1;
    }
    
    
    return 0;
}

int vl53l1x_get_distance_mode(vl53l1x_t *dev, uint16_t *distance_mode) {
    VL53L1X_GetDistanceMode(dev->params.addr, distance_mode);
    
    return 0;
}

int vl53l1x_set_inter_measurements(vl53l1x_t *dev, uint32_t inter_measurement_ms) {
    uint16_t timing_budget;

    VL53L1X_GetTimingBudgetInMs(dev->params.addr, &timing_budget);

    if (inter_measurement_ms < timing_budget) {
        inter_measurement_ms = timing_budget;
        // raise error
    }
    VL53L1X_SetInterMeasurementInMs(dev->params.addr, inter_measurement_ms);

    return 0; 
}

int vl53l1x_get_inter_measurements(vl53l1x_t *dev, uint16_t *inter_measurement_ms) {
    VL53L1X_GetInterMeasurementInMs(dev->params.addr, inter_measurement_ms);

    return 0; 
}

int vl53l1x_get_boot_state(vl53l1x_t *dev, uint8_t *state) {
    VL53L1X_BootState(dev->params.addr, state);

    return 0;
}

int vl53l1x_get_sensor_id(vl53l1x_t *dev, uint16_t *id) {
    VL53L1X_GetSensorId(dev->params.addr, id);

    return 0;
}

int vl53l1x_get_distance(vl53l1x_t *dev, uint16_t *distance) {
    VL53L1X_GetDistance(dev->params.addr, distance);

    return 0;
}

int vl53l1x_get_signal_per_spad(vl53l1x_t *dev, uint16_t *signal_per_sp) {
    VL53L1X_GetSignalPerSpad(dev->params.addr, signal_per_sp);

    return 0;
}

int vl53l1x_get_ambient_per_spad(vl53l1x_t *dev, uint16_t *amb) {
    VL53L1X_GetSignalPerSpad(dev->params.addr, amb);

    return 0;
}

int vl53l1x_get_signal_rate(vl53l1x_t *dev, uint16_t *signal_rate) {
    VL53L1X_GetSignalRate(dev->params.addr, signal_rate);

    return 0;
}

int vl53l1x_get_span_nb(vl53l1x_t *dev, uint16_t *span_nb) {
    VL53L1X_GetSpadNb(dev->params.addr, span_nb);

    return 0;
}

int vl53l1x_get_ambient_rate(vl53l1x_t *dev, uint16_t *amb_rate) {
    VL53L1X_GetAmbientRate(dev->params.addr, amb_rate);

    return 0;
}

int vl53l1x_get_range_status(vl53l1x_t *dev, uint8_t *range_status) {
    VL53L1X_GetRangeStatus(dev->params.addr, range_status);
    // to ddo with 
    return 0;
}

int vl53l1x_get_result(vl53l1x_t *dev, vl53l1x_result_t *result) {
    VL53L1X_GetResult(dev->params.addr, (VL53L1X_Result_t *)result);

    return 0;
}


int vl53l1x_set_signal_threshold(vl53l1x_t *dev, uint16_t signal) {
    VL53L1X_SetSignalThreshold(dev->params.addr, signal);

    return 0;
}

int vl53l1x_get_signal_threshold(vl53l1x_t *dev, uint16_t *signal) {
    VL53L1X_GetSignalThreshold(dev->params.addr, signal);

    return 0;
}

int vl53l1x_set_sigma_threshold(vl53l1x_t *dev, uint16_t sigma) {
    VL53L1X_SetSigmaThreshold(dev->params.addr, sigma);

    return 0;
}

int vl53l1x_get_sigma_threshold(vl53l1x_t *dev, uint16_t *sigma) {
    VL53L1X_GetSigmaThreshold(dev->params.addr, sigma);

    return 0;
}

int vl53l1x_set_distance_threshold(vl53l1x_t *dev, vl53l1x_detection_threshold_t *threshold_mode) {
    VL53L1X_SetDistanceThreshold(
        dev->params.addr,
        threshold_mode->threshold_low,
        threshold_mode->threshold_high,
        threshold_mode->window,
        INT_ON_NO_TARGET
    );

    return 0;
}

int vl53l1x_get_distance_threashold_window(vl53l1x_t *dev, uint16_t *window) {
    VL53L1X_GetDistanceThresholdWindow(dev->params.addr, window);

    return 0;
}

int vl53l1x_get_distance_threashold_high(vl53l1x_t *dev, uint16_t *threshold_low) {
    VL53L1X_GetDistanceThresholdLow(dev->params.addr, threshold_low);
    
    return 0;
}

int vl53l1x_get_distance_threashold_low(vl53l1x_t *dev, uint16_t *threshold_high) {
    VL53L1X_GetDistanceThresholdHigh(dev->params.addr, threshold_high);
    
    return 0;
}

int vl53l1x_set_roi(vl53l1x_t *dev, uint16_t x, uint16_t y) {
    VL53L1X_SetROI(dev->params.addr, x, y);
    // to do check

    return 0;
}

int vl53l1x_get_roi_xy(vl53l1x_t *dev, uint16_t *x, uint16_t *y) {
    VL53L1X_GetROI_XY(dev->params.addr, x, y);

    return 0;
}

int vl53l1x_set_roi_center(vl53l1x_t *dev, uint8_t roi_center) {
    VL53L1X_SetROICenter(dev->params.addr, roi_center);

    return 0;
}

int vl53l1x_get_roi_center(vl53l1x_t *dev, uint8_t *roi_center) {
    VL53L1X_GetROICenter(dev->params.addr, roi_center);

    return 0;
}


// calibration segment

int vl53l1x_set_offset(vl53l1x_t *dev, int16_t offset_value) {
    VL53L1X_SetOffset(dev->params.addr, offset_value);

    return 0;
}

int vl53l1x_get_offset(vl53l1x_t *dev, int16_t *offset) {
    VL53L1X_GetOffset(dev->params.addr, offset);

    return 0;
}

int vl53l1x_set_xtalk(vl53l1x_t *dev, uint16_t xtalk_value) {
    VL53L1X_SetXtalk(dev->params.addr, xtalk_value);

    return 0;
}

int vl53l1x_get_xtalk(vl53l1x_t *dev, uint16_t *xtalk) {
    VL53L1X_GetXtalk(dev->params.addr, xtalk);

    return 0;
}

int vl53l1x_start_temperature_update(vl53l1x_t *dev) {
    VL53L1X_StartTemperatureUpdate(dev->params.addr);

    return 0;
}
