/**
 * Created on: Mar 24, 2021
 * Created by: trowbridgec
 *
 * Built with ARM Mbed-OS
 * 
 * Copyright (c) Embedded Planet, Inc - All rights reserved
 *
 * This source file is private and confidential.
 * Unauthorized copying of this file is strictly prohibited.
 */

#ifndef AGORA_BSP_H
#define AGORA_BSP_H

#include "mbed.h"
#include "hal/analogin_api.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "nrfx_saadc.h"

// System voltage should be 3.3V, with the voltage divider cutting the voltage in half
static const float SYSTEM_VOLTAGE_SCALAR = 3.3 * 2;
static const uint32_t REQUIRED_SAMPLES = pow(2, SAADC_CONFIG_OVERSAMPLE);

void agora_bsp_init();
void agora_bsp_save_pin_states();
void agora_bsp_sleep();
void agora_bsp_wakeup();
void agora_bsp_led_enable(bool enabled);
void agora_bsp_cell_enable(bool enabled);
void agora_bsp_sensor_power_enable(bool enabled);
float agora_bsp_read_battery_voltage();
void agora_bsp_lora_module_sleep();

#endif // AGORA_BSP_H