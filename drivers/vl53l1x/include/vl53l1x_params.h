/*
 * Copyright (C) 2021 AUTH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_vl53l1x
 * @{
 *
 * @file
 * @brief       Register and bit definitions for the VL53L1X Time of Flight Sensor
 *
 * @author      Evripidis Chondromatidis <eurichon1996@gmail.com>
 */

#ifndef VL53L1X_REGS_H
#define VL53L1X_REGS_H

#include "board.h"
#include "vl53l1x.h"


#ifdef __cplusplus
 extern "C" {
#endif

#ifndef VL53L1X_PARAM_I2C
#define VL53L1X_PARAM_I2C           I2C_DEV(0)
#endif

#ifndef VL53L1X_PARAM_ADDR
#define VL53L1X_PARAM_ADDR          (VL53L1X_HW_ADDR_HEX_29)
#endif





#ifndef VL53L1X_PARAMS
#define VL53L1X_PARAMS            { .i2c_bus     = VL53L1X_PARAM_I2C,       \
                                    .addr        = VL53L1X_PARAM_ADDR       }
#endif

#ifndef VL53L1X_SAUL_INFO
#define VL53L1X_SAUL_INFO         { .name = "vl53l1x" }
#endif



#ifdef __cplusplus
}
#endif

#endif /* VL53L1X_REGS_H */
/** @} */
