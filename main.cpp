// ----------------------------------------------------------------------------
// Copyright 2016-2020 ARM Ltd.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
#ifndef MBED_TEST_MODE
#include "mbed.h"

#include "LSM9DS1.h"
#include "icm20602_i2c.h"
#include "Si7021.h"
#include "BME680_BSEC.h"
#include "VL53L0X.h"

EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread t;

DigitalOut sensor_power_enable(PIN_NAME_SENSOR_POWER_ENABLE);

I2C i2c(PIN_NAME_SDA, PIN_NAME_SCL);
DevI2C devi2c(PIN_NAME_SDA, PIN_NAME_SCL);

static const uint8_t LSM9DS1_ACCEL_GYRO_ADDRESS = 0x6A << 1;
static const uint8_t LSM9DS1_MAG_ADDRESS = 0x1C << 1;
LSM9DS1 lsm9ds1(i2c, LSM9DS1_ACCEL_GYRO_ADDRESS, LSM9DS1_MAG_ADDRESS);
ICM20602 icm20602(i2c);
Si7021 si7021(i2c);
BME680_BSEC *bme680 = BME680_BSEC::get_instance();
static const uint8_t VL53L0X_ADDRESS = 0x52;
VL53L0X vl53l0x(&devi2c, PIN_NAME_INT_LIGHT_TOF, VL53L0X_ADDRESS);

DigitalOut status_led(PIN_NAME_LED_RED);

float normalize_acceleration_value(int16_t acceleration_value)
{
    return acceleration_value * aRes;
}

float normalize_gyroscope_value(int16_t gyroscope_value)
{
    return gyroscope_value * gRes;
}

void update_resources(void)
{
    // Update LSM9DS1 values
    lsm9ds1.readAccel();
    lsm9ds1.readGyro();
    lsm9ds1.readMag();

    // Update SI7021 values
    si7021.measure();

    // Update VL53L0X value
    uint32_t distance;
    VL53L0X_Error retcode = vl53l0x.get_distance(&distance);
    switch (retcode) {
        case VL53L0X_ERROR_NONE:
            // Successful read
            break;
        case VL53L0X_ERROR_RANGE_ERROR:
            // Range error, return -1
            printf("VL53L0X ERROR!");
            break;
        default:
            break;
    }

    printf("--------------------------------------------------------------------------------\n");

    printf("- LSM9DS1 Accelerometer X                  : %0.2f g\n", lsm9ds1.calcAccel(lsm9ds1.ax));
    printf("- LSM9DS1 Accelerometer Y                  : %0.2f g\n", lsm9ds1.calcAccel(lsm9ds1.ay));
    printf("- LSM9DS1 Accelerometer Z                  : %0.2f g\n", lsm9ds1.calcAccel(lsm9ds1.az));
    printf("- LSM9DS1 Gyroscope X                      : %0.2f dps\n", lsm9ds1.calcGyro(lsm9ds1.gx));
    printf("- LSM9DS1 Gyroscope Y                      : %0.2f dps\n", lsm9ds1.calcGyro(lsm9ds1.gy));
    printf("- LSM9DS1 Gyroscope Z                      : %0.2f dps\n", lsm9ds1.calcGyro(lsm9ds1.gz));
    printf("- LSM9DS1 Magnetometer X                   : %0.2f gauss\n", lsm9ds1.calcMag(lsm9ds1.mx));
    printf("- LSM9DS1 Magnetometer Y                   : %0.2f gauss\n", lsm9ds1.calcMag(lsm9ds1.my));
    printf("- LSM9DS1 Magnetometer Z                   : %0.2f gauss\n", lsm9ds1.calcMag(lsm9ds1.mz));
    printf("- ICM20602 Accelerometer X                 : %0.2f g\n", normalize_acceleration_value(icm20602.getAccXvalue()));
    printf("- ICM20602 Accelerometer Y                 : %0.2f g\n", normalize_acceleration_value(icm20602.getAccYvalue()));
    printf("- ICM20602 Accelerometer Z                 : %0.2f g\n", normalize_acceleration_value(icm20602.getAccZvalue()));
    printf("- ICM20602 Gyroscope X                     : %0.2f dps\n", normalize_gyroscope_value(icm20602.getGyrXvalue()));
    printf("- ICM20602 Gyroscope Y                     : %0.2f dps\n", normalize_gyroscope_value(icm20602.getGyrYvalue()));
    printf("- ICM20602 Gyroscope Z                     : %0.2f dps\n", normalize_gyroscope_value(icm20602.getGyrZvalue()));
    printf("- SI7021 Temperature                       : %0.2f degC\n", si7021.get_temperature() / 1000.0);
    printf("- SI7021 Relative Humidity                 : %0.2f %%RH\n", si7021.get_humidity() / 1000.0);
    printf("- BME680 Temperature                       : %0.2f degC\n", bme680->get_temperature());
    printf("- BME680 Relative Humidity                 : %0.2f %%RH\n", bme680->get_humidity());
    printf("- BME680 Pressure                          : %0.2f kPa\n", bme680->get_pressure() / 1000.0);
    printf("- BME680 Gas Resistance                    : %0.2f kOhms\n", bme680->get_gas_resistance() / 1000.0);
    printf("- BME680 CO2 Equivalents                   : %0.2f ppm\n", bme680->get_co2_equivalent());
    printf("- BME680 Breath-VOC Equivalents            : %0.2f ppm\n", bme680->get_breath_voc_equivalent());

    float iaq_score = bme680->get_iaq_score();
    if (iaq_score < 51.0) {
        // Good
        printf("- BME680 IAQ Score                         : %0.2f (Good)\n", iaq_score);
    } else if (iaq_score >= 51.0 && iaq_score < 101.0 ) {
        // Average
        printf("- BME680 IAQ Score                         : %0.2f (Average)\n", iaq_score);
    } else if (iaq_score >= 101.0 && iaq_score < 151.0 ) {
        // Little bad
        printf("- BME680 IAQ Score                         : %0.2f (Little bad)\n", iaq_score);
    } else if (iaq_score >= 151.0 && iaq_score < 201.0 ) {
        // Bad
        printf("- BME680 IAQ Score                         : %0.2f (Bad)\n", iaq_score);
    } else if (iaq_score >= 201.0 && iaq_score < 301.0 ) {
        // Worse
        printf("- BME680 IAQ Score                         : %0.2f (Worse)\n", iaq_score);
    } else {
        // Very bad
        printf("- BME680 IAQ Score                         : %0.2f (Very bad)\n", iaq_score);
    }

    switch (bme680->get_iaq_accuracy()) {
        default:
        case 0:
            printf("- BME680 IAQ Accuracy                      : 0 (Unreliable)\n");
            break;
        case 1:
            printf("- BME680 IAQ Accuracy                      : 1 (Low accuracy)\n");
            break;
        case 2:
            printf("- BME680 IAQ Accuracy                      : 2 (Medium accuracy)\n");
            break;
        case 3:
            printf("- BME680 IAQ Accuracy                      : 3 (High accuracy)\n");
            break;
    }
    if (distance >= 0) {
        printf("- VL53L0X Ranging Time-of-Flight Distance  : %0.2f m\n", distance / 1000.0);
    } else {
        printf("- VL53L0X Ranging Time-of-Flight Distance  : OUT OF RANGE\n");
    }

    printf("--------------------------------------------------------------------------------\n\n");
}

int main(void)
{
    printf("Enable power to the sensors\n");
    sensor_power_enable = 1;

    // Sleep to give sensors time to come online
    ThisThread::sleep_for(500);

    if (lsm9ds1.begin()) {
        printf("LSM9DS1 online\n");
        lsm9ds1.calibrate();
    } else {
        printf("ERROR: LSM9DS1 offline!\n");
    }

    if (icm20602.isOnline()) {
        printf("ICM20602 online\n");
        icm20602.init();
    } else {
        printf("ERROR: ICM20602 offline!\n");
    }

    if (si7021.check()) {
        printf("SI7021 online\n");
    } else {
        printf("ERROR: SI7021 offline!\n");
    }

    if (bme680->init(&i2c)) {
        printf("BME680 online\n");
    } else {
        printf("ERROR: BME680 offline!\n");
    }

    VL53L0X_Error retcode = vl53l0x.init_sensor(VL53L0X_ADDRESS);
    if (retcode == VL53L0X_ERROR_NONE) {
        printf("VL53L0X online\n");
    } else {
        printf("ERROR: VL53L0X offline!\n");
    }

    t.start(callback(&queue, &EventQueue::dispatch_forever));
    queue.call_every(30000, update_resources);

    return 0;
}

#endif /* MBED_TEST_MODE */
