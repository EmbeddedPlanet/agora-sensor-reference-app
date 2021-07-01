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

#include "agora_bsp.h"
#include "mbed_power_mgmt.h"

// DEFINES

#define NRF_LF_CLK_MODE CLOCK_LFRCMODE_MODE_ULP

#define NRF52_ONRAM1_OFFRAM1    POWER_RAM_POWER_S0POWER_On      << POWER_RAM_POWER_S0POWER_Pos      \
                              | POWER_RAM_POWER_S1POWER_On      << POWER_RAM_POWER_S1POWER_Pos      \
                              | POWER_RAM_POWER_S0RETENTION_On  << POWER_RAM_POWER_S0RETENTION_Pos  \
                              | POWER_RAM_POWER_S1RETENTION_On  << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM1_OFFRAM0    POWER_RAM_POWER_S0POWER_On      << POWER_RAM_POWER_S0POWER_Pos      \
                              | POWER_RAM_POWER_S1POWER_On      << POWER_RAM_POWER_S1POWER_Pos      \
                              | POWER_RAM_POWER_S0RETENTION_Off << POWER_RAM_POWER_S0RETENTION_Pos  \
                              | POWER_RAM_POWER_S1RETENTION_Off << POWER_RAM_POWER_S1RETENTION_Pos;

#define NRF52_ONRAM0_OFFRAM0    POWER_RAM_POWER_S0POWER_Off     << POWER_RAM_POWER_S0POWER_Pos      \
                              | POWER_RAM_POWER_S1POWER_Off     << POWER_RAM_POWER_S1POWER_Pos;

// LOCAL VARIABLES
static bool sleep_flag = false;
static uint32_t pin_states[2][32];
static uint32_t gpiote_config0;
static uint32_t gpiote_int;
static uint32_t ppi_ch;

// LOCAL HELPER FUNCTIONS

void configure_ram_retention(void)
{
    // Configure nRF52 RAM retention parameters. Set for System On 64kB RAM retention
    NRF_POWER->RAM[0].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[1].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[2].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[3].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[4].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[5].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[6].POWER = NRF52_ONRAM1_OFFRAM0;
    NRF_POWER->RAM[7].POWER = NRF52_ONRAM1_OFFRAM0;
}

void exit_sleep()
{
    sleep_flag = true;
}

void set_adc_acquisition_time_to_40us(PinName pin)
{
    /* Use pinmap function to get associated channel. */
    uint32_t channel = pinmap_function(pin, PinMap_ADC);

    /* Account for an off-by-one in Channel definition and Input definition. */
    nrf_saadc_input_t input = (nrf_saadc_input_t)(channel + 1);

    /* Configure channel and pin:
     *  - the 1/4 gain and VDD/4 makes the reference voltage VDD.
     */
    nrf_saadc_channel_config_t channel_config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_4,
        .reference  = NRF_SAADC_REFERENCE_VDD4,
        .acq_time   = NRF_SAADC_ACQTIME_40US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_DISABLED,
        .pin_p      = input,
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    // First, uninit the channel
    ret_code_t result = nrfx_saadc_channel_uninit(channel);
    if (result != NRFX_SUCCESS) {
        printf("Error setting acquisition time to 40us - could not uninit the channel");
        return;
    }

    // Then, re-init the channel with the new configuration
    result = nrfx_saadc_channel_init(channel, &channel_config);
    if (result != NRFX_SUCCESS) {
        printf("Error setting acquisition time to 40us - could not init the channel");
    }
}

bool is_protected_pin(int pin_number)
{
    return pin_number == 17 || pin_number == 19 || pin_number == 20 || pin_number == 21 || pin_number == 33 || pin_number == 34;
    // return pin_number == 17 || pin_number == 19 || pin_number == 20 || pin_number == 21 || pin_number == 22 || pin_number == 23 || pin_number == 29 || pin_number == 26 || pin_number == 27 || pin_number == 8;
}

// EXPOSED FUNCTIONS

void agora_bsp_init()
{
    NRF_POWER->DCDCEN = 1;
    agora_bsp_lora_module_sleep();
    NRF_P0->PIN_CNF[28] = 0x00000003;
    NRF_P0->PIN_CNF[30] = 0x00000003;
    agora_bsp_save_pin_states();
}

void agora_bsp_save_pin_states()
{
    for (int i = 0; i < 32; i++) { //SKIP TURNING OFF SENSOR POWER ENABLE ON 0_31, and also the accel interrupt on 1_5, and push button interrupt on 0_29,
        pin_states[0][i] = NRF_P0->PIN_CNF[i];
    }
    for (int i = 0; i < 32; i++) {
        pin_states[1][i] = NRF_P0->PIN_CNF[i];
    }
    ppi_ch = NRF_PPI->CHEN;
}

void agora_bsp_sleep()
{
    //Configure RAM retention. More RAM retention means increased current consumption (see electrical specification in the Product Specification, power chapter)
    configure_ram_retention();

    for (int i = 0; i < 32; i++) { //SKIP TURNING OFF SENSOR POWER ENABLE ON 0_31, and also the accel interrupt on 1_5, and push button interrupt on 0_29,
        if (is_protected_pin(i)) {
            continue;
        }
        pin_states[0][i] = NRF_P0->PIN_CNF[i];
        NRF_P0->PIN_CNF[i] = 0x00000002;
    }
    for (int i = 0; i < 32; i++) {
        pin_states[1][i] = NRF_P0->PIN_CNF[i];
        NRF_P1->PIN_CNF[i] = 0x00000002;
    }

    // NRF_SPI0->ENABLE = 0;
    // NRF_SPI1->ENABLE = 0;

    NRF_TWI0->ENABLE = 0;
    /*
        // mbed_file_handle(STDIN_FILENO)->enable_input(false);
        NRF_UART0->TASKS_STOPTX = 1;
        NRF_UART0->TASKS_STOPRX = 1;
        NRF_UART0->ENABLE = 0;
        NRF_UARTE0->TASKS_STOPTX = 1;
        NRF_UARTE0->TASKS_STOPRX = 1;
        NRF_UARTE0->ENABLE = 0;
        NRF_UARTE1->TASKS_STOPTX = 1;
        NRF_UARTE1->TASKS_STOPRX = 1;
        NRF_UARTE1->ENABLE = 0;
    */
    // NRF_CLOCK->LFRCMODE             = (NRF_LF_CLK_MODE << CLOCK_LFRCMODE_MODE_Pos);
    // NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;
    // NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    // while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {
    // }
    // NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

    NRF_CRYPTOCELL->ENABLE = 0; // disable

    NRF_UICR->NFCPINS = 0; // disable

    NRF_RADIO->POWER = 0; // turn off

    NRF_SAADC->ENABLE = 0; //disable

    // NRF_SPI2->ENABLE = 0;

    // NRF_TIMER0->MODE = 2; // low power counter
    // NRF_TIMER0->INTENCLR = 0xffffffff; // disable all interrupts
    // NRF_TIMER1->MODE = 2; // low power counter
    // NRF_TIMER1->INTENCLR = 0xffffffff; // disable all interrupts
    // NRF_TIMER2->MODE = 2; // low power counter
    // NRF_TIMER2->INTENCLR = 0xffffffff; // disable all interrupts
    // NRF_TIMER3->MODE = 2; // low power counter
    // NRF_TIMER3->INTENCLR = 0xffffffff; // disable all interrupts
    // NRF_TIMER4->MODE = 2; // low power counter
    // NRF_TIMER4->INTENCLR = 0xffffffff; // disable all interrupts

    NRF_UICR->PSELRESET[0] = 0x80000012; //disconnected
    NRF_UICR->PSELRESET[1] = 0xffffffff; //disconnected

    NRF_USBD->LOWPOWER = 1; // Force low power

    NRF_WDT->CONFIG = 0; // disable
    NRF_WDT->RREN = 0; // disable

    // gpiote_config0 = NRF_GPIOTE->CONFIG[0];
    // gpiote_int = NRF_GPIOTE->INTENSET;
    // NRF_GPIOTE->CONFIG[0] = 0; // disable
    // NRF_GPIOTE->INTENCLR = 0xffffffff; // disable all interrupts

    ppi_ch = NRF_PPI->CHEN;
    NRF_PPI->CHENCLR = 0xffffffff; // disable all channels

    // while (!sleep_manager_can_deep_sleep()) {
    //     sleep_manager_unlock_deep_sleep();
    // }
}

void agora_bsp_wakeup()
{
    for (int i = 0; i < 32; i++) { //SKIP TURNING OFF SENSOR POWER ENABLE ON 0_31, and also the accel interrupt on 1_5, and push button interrupt on 0_29,
        if (is_protected_pin(i)) {
            continue;
        }
        NRF_P0->PIN_CNF[i] = pin_states[0][i];
    }
    for (int i = 0; i < 32; i++) {
        NRF_P1->PIN_CNF[i] = pin_states[1][i];
    }

    // NRF_SPI0->ENABLE = 1;
    // NRF_SPI1->ENABLE = 1;
    // NRF_SPI2->ENABLE = 1;

    // NRF_TWI0->ENABLE = 1;
    /*
        NRF_UART0->ENABLE = 1;
        NRF_UART0->TASKS_STARTTX = 1;
        NRF_UART0->TASKS_STARTRX = 1;
        NRF_UARTE0->ENABLE = 1;
        NRF_UARTE0->TASKS_STARTTX = 1;
        NRF_UARTE0->TASKS_STARTRX = 1;
        NRF_UARTE1->ENABLE = 1;
        NRF_UARTE1->TASKS_STARTTX = 1;
        NRF_UARTE1->TASKS_STARTRX = 1;
    */
    NRF_CRYPTOCELL->ENABLE = 1; // enable

    // NRF_GPIOTE->CONFIG[0] = gpiote_config0;
    // NRF_GPIOTE->INTENSET = gpiote_int;

    NRF_PPI->CHENSET = ppi_ch;
}

void agora_bsp_led_enable(bool enabled)
{
    DigitalOut led(PIN_NAME_LED_RED);
    led = !enabled;
}

void agora_bsp_cell_enable(bool enabled)
{
    DigitalOut cell_power_enable(PIN_NAME_CELL_POWER_ENABLE);
    cell_power_enable = enabled;
}

void agora_bsp_sensor_power_enable(bool enabled)
{
    DigitalOut sensor_power_enable(PIN_NAME_SENSOR_POWER_ENABLE);
    sensor_power_enable = enabled;
}

float agora_bsp_read_battery_voltage()
{
    AnalogIn battery_voltage_internal(PIN_NAME_BATTERY);
    DigitalOut battery_monitor_enable_internal(PIN_NAME_BATTERY_MONITOR_ENABLE);
    float battery_voltage_value_internal = 0.0;

    // Enable the ADC
    NRF_SAADC->ENABLE = 1;

    // Re-configure ADC to use 40us acquisition time
    set_adc_acquisition_time_to_40us(PIN_NAME_BATTERY);

    // Enable battery monitoring
    battery_monitor_enable_internal = 1;

    // Take the ADC reading
    for (uint32_t i = 0; i < REQUIRED_SAMPLES; i++) {
        battery_voltage_value_internal = battery_voltage_internal.read();
    }

    // Disable battery monitoring
    battery_monitor_enable_internal = 0;

    // Return normalized value
    battery_voltage_value_internal = battery_voltage_value_internal * SYSTEM_VOLTAGE_SCALAR;

    // Disable the ADC
    NRF_SAADC->ENABLE = 0;

    return battery_voltage_value_internal;
}

void agora_bsp_lora_module_sleep()
{
    mbed::SPI _spi(PIN_NAME_LORA_MOSI, PIN_NAME_LORA_MISO, PIN_NAME_LORA_SCLK);
    mbed::DigitalOut _chip_select(PIN_NAME_LORA_SSN, 1);
    mbed::DigitalInOut _reset_ctl(PIN_NAME_LORA_RESETN);
    const uint8_t AGORA_BSP_LORA_REG_OPMODE     = 0x01;
    const uint8_t AGORA_BSP_LORA_SPI_READ_CMD   = 0x7F;
    const uint8_t AGORA_BSP_LORA_SPI_WRITE_CMD  = 0x80;
    const uint8_t AGORA_BSP_LORA_RF_OPMODE_MASK = 0xF8;

    /* Reset the radio */
    _reset_ctl.output();
    _reset_ctl = 0;
    ThisThread::sleep_for(2);
    _reset_ctl.input();
    ThisThread::sleep_for(6);

    /* Setup SPI */
    // SPI bus frequency
    uint32_t spi_freq = 8000000;

    // Hold chip-select high
    _chip_select = 1;
    _spi.format(8, 0);
    _spi.frequency(spi_freq);

    // 100 us wait to settle down
    ThisThread::sleep_for(0.1);

    /* Put to sleep */
    // Read current state
    uint8_t buffer;
    // set chip-select low
    _chip_select = 0;

    // set read command
    _spi.write(AGORA_BSP_LORA_REG_OPMODE & AGORA_BSP_LORA_SPI_READ_CMD);

    // read buffers
    buffer = _spi.write(0);

    // set chip-select high
    _chip_select = 1;

    // Add OPMODE mask
    buffer = buffer & AGORA_BSP_LORA_RF_OPMODE_MASK;

    // set chip-select low
    _chip_select = 0;

    // set write command
    _spi.write(AGORA_BSP_LORA_REG_OPMODE | AGORA_BSP_LORA_SPI_WRITE_CMD);

    // write data
    _spi.write(buffer);

    // set chip-select high
    _chip_select = 1;
}