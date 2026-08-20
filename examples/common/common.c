/**
 * Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    common.c
 * @brief   Common source file for the BHy examples
 *
 * Note: Parts of the code in this file are from GenAI GitHub Copilot
 */

#include "common.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "bhi385_parse.h"

#define ROBERT_BOSCH_USB_VID   (0x108C)
#define ARDUINO_USB_VID        (0x2341)
#define BST_APP31_CDC_USB_PID  (0xAB38)
#define BST_APP30_CDC_USB_PID  (0xAB3C)
#define BST_APP20_CDC_USB_PID  (0xAB2C)
#define ARDUINO_NICLA_USB_PID  (0x0060)

void verbose_write(uint8_t *buffer, uint16_t length);

#ifdef MCU_APP20
static enum coines_multi_io_pin cs_pin = BHY385_APP20_CS_PIN;
static enum coines_multi_io_pin int_pin = BHY385_APP20_INT_PIN;
static enum coines_multi_io_pin reset_pin = BHY385_APP20_RESET_PIN;
#else
static enum coines_multi_io_pin cs_pin = BHY385_APP30_CS_PIN;
static enum coines_multi_io_pin int_pin = BHY385_APP30_INT_PIN;
static enum coines_multi_io_pin reset_pin = BHY385_APP30_RESET_PIN;
#endif

/*!
 * @brief Generic lookup table entry mapping an integer key to a string value.
 *        Used to replace long switch-case chains with simple table lookups,
 *        keeping the lookup functions at a constant, low cyclomatic complexity.
 */
typedef struct
{
    int32_t key;
    char *value;
} bhi385_str_lut_entry;

/*!
 * @brief Generic lookup table entry mapping an integer key to a uint8_t value.
 */
typedef struct
{
    int32_t key;
    uint8_t value;
} bhi385_u8_lut_entry;

/*!
 * @brief Looks up a string value for the given key in a lookup table.
 *
 * @param[in] table         Lookup table to search.
 * @param[in] table_size    Number of entries in the table.
 * @param[in] key           Key to search for.
 * @param[in] default_value Value returned when the key is not found.
 *
 * @return The matching value, or default_value if the key is not present.
 */
static char *bhi385_str_lut_lookup(const bhi385_str_lut_entry *table,
                                   size_t table_size,
                                   int32_t key,
                                   char *default_value)
{
    size_t i;

    for (i = 0; i < table_size; i++)
    {
        if (table[i].key == key)
        {
            return table[i].value;
        }
    }

    return default_value;
}

/*!
 * @brief Looks up a uint8_t value for the given key in a lookup table.
 *
 * @param[in] table         Lookup table to search.
 * @param[in] table_size    Number of entries in the table.
 * @param[in] key           Key to search for.
 * @param[in] default_value Value returned when the key is not found.
 *
 * @return The matching value, or default_value if the key is not present.
 */
static uint8_t bhi385_u8_lut_lookup(const bhi385_u8_lut_entry *table,
                                    size_t table_size,
                                    int32_t key,
                                    uint8_t default_value)
{
    size_t i;

    for (i = 0; i < table_size; i++)
    {
        if (table[i].key == key)
        {
            return table[i].value;
        }
    }

    return default_value;
}

bool get_interrupt_status(void)
{
    int16_t coines_rslt;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;

    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_get_pin_config(int_pin, &pin_direction, &pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        char *error_text = get_coines_error(coines_rslt);
        printf("Error getting interrupt pin status %s\r\n", error_text);
    }

    return pin_value == COINES_PIN_VALUE_HIGH;
}

char *get_coines_error(int16_t rslt)
{
    static const bhi385_str_lut_entry table[] = {
        { COINES_SUCCESS, " " }, { COINES_E_FAILURE, "[COINES Error] Generic failure" },
        { COINES_E_COMM_IO_ERROR, "[COINES Error] Communication IO failed. Check connections with the sensor" },
        { COINES_E_COMM_INIT_FAILED, "[COINES Error] Communication initialization failed" },
        { COINES_E_UNABLE_OPEN_DEVICE, "[COINES Error] Unable to open device. Check if the board is in use" },
        { COINES_E_DEVICE_NOT_FOUND, "[COINES Error] Device not found. Check if the board is powered on" },
        { COINES_E_UNABLE_CLAIM_INTF, "[COINES Error] Unable to claim interface. Check if the board is in use" },
        { COINES_E_MEMORY_ALLOCATION, "[COINES Error] Error allocating memory" },
        { COINES_E_NOT_SUPPORTED, "[COINES Error] Feature not supported" },
        { COINES_E_NULL_PTR, "[COINES Error] Null pointer error" },
        { COINES_E_COMM_WRONG_RESPONSE, "[COINES Error] Unexpected response" },
        { COINES_E_SPI16BIT_NOT_CONFIGURED, "[COINES Error] 16-Bit SPI not configured" },
        { COINES_E_SPI_INVALID_BUS_INTF, "[COINES Error] Invalid SPI bus interface" },
        { COINES_E_SPI_CONFIG_EXIST, "[COINES Error] SPI already configured" },
        { COINES_E_SPI_BUS_NOT_ENABLED, "[COINES Error] SPI bus not enabled" },
        { COINES_E_SPI_CONFIG_FAILED, "[COINES Error] SPI configuration failed" },
        { COINES_E_I2C_INVALID_BUS_INTF, "[COINES Error] Invalid I2C bus interface" },
        { COINES_E_I2C_BUS_NOT_ENABLED, "[COINES Error] I2C bus not enabled" },
        { COINES_E_I2C_CONFIG_FAILED, "[COINES Error] I2C configuration failed" },
        { COINES_E_I2C_CONFIG_EXIST, "[COINES Error] I2C already configured" },
    };

    return bhi385_str_lut_lookup(table, sizeof(table) / sizeof(table[0]), rslt, "[COINES Error] Unknown error code");
}

char *get_api_error(int8_t error_code)
{
    char *ret = " ";

    switch (error_code)
    {
        case BHI385_OK:
            break;
        case BHI385_E_NULL_PTR:
            ret = "[API Error] Null pointer";
            break;
        case BHI385_E_INVALID_PARAM:
            ret = "[API Error] Invalid parameter";
            break;
        case BHI385_E_IO:
            ret = "[API Error] IO error";
            break;
        case BHI385_E_MAGIC:
            ret = "[API Error] Invalid firmware";
            break;
        case BHI385_E_TIMEOUT:
            ret = "[API Error] Timed out";
            break;
        case BHI385_E_BUFFER:
            ret = "[API Error] Invalid buffer";
            break;
        case BHI385_E_INVALID_FIFO_TYPE:
            ret = "[API Error] Invalid FIFO type";
            break;
        case BHI385_E_INVALID_EVENT_SIZE:
            ret = "[API Error] Invalid Event size";
            break;
        case BHI385_E_PARAM_NOT_SET:
            ret = "[API Error] Parameter not set";
            break;
        default:
            ret = "[API Error] Unknown API error code";
    }

    return ret;
}

void setup_interfaces(bool reset_power, enum bhi385_intf intf)
{
    int16_t coines_rslt = COINES_SUCCESS;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;
    struct coines_board_info board_info;
    char *error_text;

#ifndef PC
    struct coines_ble_config ble_config;
    ble_config.name = NULL;
    ble_config.tx_power = COINES_TX_POWER_8_DBM;
    coines_rslt = coines_ble_config(&ble_config);
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);
#else
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
#endif
    if (coines_rslt)
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\n", error_text);
    }

    coines_rslt = coines_get_board_info(&board_info);
    if (coines_rslt == COINES_SUCCESS)
    {
        if (board_info.board == 5) /* Application Board 3.0 */
        {
            cs_pin = BHY385_APP30_CS_PIN;
            int_pin = BHY385_APP30_INT_PIN;
            reset_pin = BHY385_APP30_RESET_PIN;
        }
    }
    else
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\n", error_text);
    }

    if (reset_power)
    {
        coines_rslt = coines_set_shuttleboard_vdd_vddio_config(0, 0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\n", error_text);
        }

        pin_direction = COINES_PIN_DIRECTION_OUT;
        pin_value = COINES_PIN_VALUE_LOW;
        coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\n", error_text);
        }

        coines_delay_msec(10);
    }

    if (intf == BHI385_SPI_INTERFACE)
    {
        printf("Host Interface : SPI\r\n");
        coines_rslt = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_1_MHZ, COINES_SPI_MODE0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to SPI %s\r\n", error_text);
        }
    }
    else
    {
        printf("Host Interface : I2C\r\n");
        coines_rslt = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to I2C %s\r\n", error_text);
        }
    }

    coines_rslt = coines_set_shuttleboard_vdd_vddio_config(1800, 1800);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting Vdd and Vddio to 1.8V %s\r\n", error_text);
    }

    pin_direction = COINES_PIN_DIRECTION_OUT;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting the reset pin %s\r\n", error_text);
    }

    /* Configure as a pull-down. The BHy operates the interrupt pin as an active high, level, push-pull by default */
    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_LOW;
    coines_rslt = coines_set_pin_config(int_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error configuring the interrupt pin %s\r\n", error_text);
    }

    coines_delay_msec(50);
}

void setup_interfaces_with_port(bool reset_power, enum bhi385_intf intf, const char *com_port)
{
    int16_t coines_rslt = COINES_SUCCESS;
    enum coines_pin_direction pin_direction;
    enum coines_pin_value pin_value;
    struct coines_board_info board_info;
    char *error_text;

#ifndef PC
    struct coines_ble_config ble_config;
    ble_config.name = NULL;
    ble_config.tx_power = COINES_TX_POWER_8_DBM;
    coines_rslt = coines_ble_config(&ble_config);
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_BLE, NULL);
#else
    struct coines_serial_com_config scom_config;
    scom_config.baud_rate = 9600;
    scom_config.vendor_id = ROBERT_BOSCH_USB_VID;
    scom_config.product_id = BST_APP30_CDC_USB_PID;
    scom_config.com_port_name = strdup(com_port);
    scom_config.rx_buffer_size = 2048;
    coines_rslt = coines_open_comm_intf(COINES_COMM_INTF_USB, &scom_config);
    free(scom_config.com_port_name); /* free memory */
#endif
    if (coines_rslt)
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\n", error_text);
    }

    coines_rslt = coines_get_board_info(&board_info);
    if (coines_rslt == COINES_SUCCESS)
    {
#ifdef PC
        if (board_info.board == 3) /* Application Board 2.0 */
        {
            scom_config.product_id = BST_APP20_CDC_USB_PID;
        }
        else if (board_info.board == 9) /* Application Board 3.1 */
        {
            scom_config.product_id = BST_APP31_CDC_USB_PID;
        }

#endif
        if (board_info.board == 5) /* Application Board 3.0 */
        {
            cs_pin = BHY385_APP30_CS_PIN;
            int_pin = BHY385_APP30_INT_PIN;
            reset_pin = BHY385_APP30_RESET_PIN;
        }
    }
    else
    {
        error_text = get_coines_error(coines_rslt);
        printf("%s\r\n", error_text);
    }

    if (reset_power)
    {
        coines_rslt = coines_set_shuttleboard_vdd_vddio_config(0, 0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\r\n", error_text);
        }

        pin_direction = COINES_PIN_DIRECTION_OUT;
        pin_value = COINES_PIN_VALUE_LOW;
        coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("%s\r\n", error_text);
        }

        coines_delay_msec(10);
    }

    if (intf == BHI385_SPI_INTERFACE)
    {
        printf("Host Interface : SPI\r\n");
        coines_rslt = coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_1_MHZ, COINES_SPI_MODE0);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to SPI %s\r\n", error_text);
        }
    }
    else
    {
        printf("Host Interface : I2C\r\n");
        coines_rslt = coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_FAST_MODE);
        if (coines_rslt != COINES_SUCCESS)
        {
            error_text = get_coines_error(coines_rslt);
            printf("Error configuring to I2C %s\r\n", error_text);
        }
    }

    coines_rslt = coines_set_shuttleboard_vdd_vddio_config(1800, 1800);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting Vdd and Vddio to 1.8V.\r\n%s\r\n", error_text);
    }

    pin_direction = COINES_PIN_DIRECTION_OUT;
    pin_value = COINES_PIN_VALUE_HIGH;
    coines_rslt = coines_set_pin_config(reset_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error setting the reset pin %s\r\n", error_text);
    }

    /* Configure as a pull-down. The BHy operates the interrupt pin as an active high, level, push-pull by default */
    pin_direction = COINES_PIN_DIRECTION_IN;
    pin_value = COINES_PIN_VALUE_LOW;
    coines_rslt = coines_set_pin_config(int_pin, pin_direction, pin_value);
    if (coines_rslt != COINES_SUCCESS)
    {
        error_text = get_coines_error(coines_rslt);
        printf("Error configuring the interrupt pin %s\r\n", error_text);
    }

    coines_delay_msec(50);
}

void close_interfaces(enum bhi385_intf intf)
{
    if (intf == BHI385_I2C_INTERFACE)
    {
        (void)coines_deconfig_i2c_bus(COINES_I2C_BUS_0);
    }
    else
    {
        (void)coines_deconfig_spi_bus(COINES_SPI_BUS_0);
    }

    (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    (void)fflush(stdout);

    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);

    coines_delay_msec(100);

    /* Coines interface reset */
    coines_soft_reset();
    coines_delay_msec(100);
}

int8_t bhi385_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_read_spi(COINES_SPI_BUS_0, cs_pin, reg_addr, reg_data, (uint16_t)length);
}

int8_t bhi385_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_write_spi(COINES_SPI_BUS_0, cs_pin, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

int8_t bhi385_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_read_i2c(COINES_I2C_BUS_0, 0x28, reg_addr, reg_data, (uint16_t)length);
}

int8_t bhi385_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    return coines_write_i2c(COINES_I2C_BUS_0, 0x28, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

void bhi385_delay_us(uint32_t us, void *private_data)
{
    (void)private_data;
    coines_delay_usec(us);
}

char *get_sensor_error_text(uint8_t sensor_error)
{
    static const bhi385_str_lut_entry table[] = {
        { 0x00, "Error code not recognized" },
        { 0x10, "[Sensor error] Bootloader reports: Firmware Expected Version Mismatch" },
        { 0x11, "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Header CRC" },
        { 0x12, "[Sensor error] Bootloader reports: Firmware Upload Failed: SHA Hash Mismatch" },
        { 0x13, "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Image CRC" },
        { 0x14, "[Sensor error] Bootloader reports: Firmware Upload Failed: ECDSA Signature Verification Failed" },
        { 0x15, "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad Public Key CRC" },
        { 0x16, "[Sensor error] Bootloader reports: Firmware Upload Failed: Signed Firmware Required" },
        { 0x17, "[Sensor error] Bootloader reports: Firmware Upload Failed: FW Header Missing" },
        { 0x19, "[Sensor error] Bootloader reports: Unexpected Watchdog Reset" },
        { 0x1A, "[Sensor error] ROM Version Mismatch" },
        { 0x1B, "[Sensor error] Bootloader reports: Fatal Firmware Error" },
        { 0x1C, "[Sensor error] Chained Firmware Error: Next Payload Not Found" },
        { 0x1D, "[Sensor error] Chained Firmware Error: Payload Not Valid" },
        { 0x1E, "[Sensor error] Chained Firmware Error: Payload Entries Invalid" },
        { 0x1F, "[Sensor error] Bootloader reports: Bootloader Error: OTP CRC Invalid" },
        { 0x20, "[Sensor error] Firmware Init Failed" },
        { 0x21, "[Sensor error] Sensor Init Failed: Unexpected Device ID" },
        { 0x22, "[Sensor error] Sensor Init Failed: No Response from Device" },
        { 0x23, "[Sensor error] Sensor Init Failed: Unknown" }, { 0x24, "[Sensor error] Sensor Error: No Valid Data" },
        { 0x25, "[Sensor error] Slow Sample Rate" }, { 0x26, "[Sensor error] Data Overflow (saturated sensor data)" },
        { 0x27, "[Sensor error] Stack Overflow" }, { 0x28, "[Sensor error] Insufficient Free RAM" },
        { 0x29, "[Sensor error] Sensor Init Failed: Driver Parsing Error" },
        { 0x2A, "[Sensor error] Too Many RAM Banks Required" }, { 0x2B, "[Sensor error] Invalid Event Specified" },
        { 0x2C, "[Sensor error] More than 32 On Change" }, { 0x2D, "[Sensor error] Firmware Too Large" },
        { 0x2F, "[Sensor error] Invalid RAM Banks" }, { 0x30, "[Sensor error] Math Error" },
        { 0x40, "[Sensor error] Memory Error" }, { 0x41, "[Sensor error] SWI3 Error" },
        { 0x42, "[Sensor error] SWI4 Error" }, { 0x43, "[Sensor error] Illegal Instruction Error" },
        { 0x44, "[Sensor error] Bootloader reports: Unhandled Interrupt Error / Exception / Postmortem Available" },
        { 0x45, "[Sensor error] Invalid Memory Access" }, { 0x50, "[Sensor error] Algorithm Error: BSX Init" },
        { 0x51, "[Sensor error] Algorithm Error: BSX Do Step" }, { 0x52, "[Sensor error] Algorithm Error: Update Sub" },
        { 0x53, "[Sensor error] Algorithm Error: Get Sub" }, { 0x54, "[Sensor error] Algorithm Error: Get Phys" },
        { 0x55, "[Sensor error] Algorithm Error: Unsupported Phys Rate" },
        { 0x56, "[Sensor error] Algorithm Error: Cannot find BSX Driver" },
        { 0x60, "[Sensor error] Sensor Self-Test Failure" }, { 0x61, "[Sensor error] Sensor Self-Test X Axis Failure" },
        { 0x62, "[Sensor error] Sensor Self-Test Y Axis Failure" },
        { 0x64, "[Sensor error] Sensor Self-Test Z Axis Failure" }, { 0x65, "[Sensor error] FOC Failure" },
        { 0x66, "[Sensor error] Sensor Busy" }, { 0x6F, "[Sensor error] Self-Test or FOC Test Unsupported" },
        { 0x72, "[Sensor error] No Host Interrupt Set" },
        { 0x73, "[Sensor error] Event ID Passed to Host Interface Has No Known Size" },
        { 0x75, "[Sensor error] Host Download Channel Underflow (Host Read Too Fast)" },
        { 0x76, "[Sensor error] Host Upload Channel Overflow (Host Wrote Too Fast)" },
        { 0x77, "[Sensor error] Host Download Channel Empty" }, { 0x78, "[Sensor error] DMA Error" },
        { 0x79, "[Sensor error] Corrupted Input Block Chain" }, { 0x7A, "[Sensor error] Corrupted Output Block Chain" },
        { 0x7B, "[Sensor error] Buffer Block Manager Error" },
        { 0x7C, "[Sensor error] Input Channel Not Word Aligned" }, { 0x7D, "[Sensor error] Too Many Flush Events" },
        { 0x7E, "[Sensor error] Unknown Host Channel Error" }, { 0x81, "[Sensor error] Decimation Too Large" },
        { 0x90, "[Sensor error] Master SPI/I2C Queue Overflow" }, { 0x91, "[Sensor error] SPI/I2C Callback Error" },
        { 0xA0, "[Sensor error] Timer Scheduling Error" }, { 0xB0, "[Sensor error] Invalid GPIO for Host IRQ" },
        { 0xB1, "[Sensor error] Error Sending Initialized Meta Events" },
        { 0xC0, "[Sensor error] Bootloader reports: Command Error" },
        { 0xC1, "[Sensor error] Bootloader reports: Command Too Long" },
        { 0xC2, "[Sensor error] Bootloader reports: Command Buffer Overflow" },
        { 0xD0, "[Sensor error] User Mode Error: Sys Call Invalid" },
        { 0xD1, "[Sensor error] User Mode Error: Trap Invalid" },
        { 0xE1, "[Sensor error] Firmware Upload Failed: Firmware header corrupt" },
        { 0xE2, "[Sensor error] Sensor Data Injection: Invalid input stream" },
    };

    return bhi385_str_lut_lookup(table,
                                 sizeof(table) / sizeof(table[0]),
                                 sensor_error,
                                 "[Sensor error] Unknown error code");
}

char *get_physical_sensor_name(uint8_t sensor_id)
{
    static const bhi385_str_lut_entry table[] = {
        { BHI385_PHYS_SENSOR_ID_ACCELEROMETER, "Accelerometer" },
        { BHI385_PHYS_SENSOR_ID_NOT_SUPPORTED, "Not supported now" }, { BHI385_PHYS_SENSOR_ID_GYROSCOPE, "Gyroscope" },
        { BHI385_PHYS_SENSOR_ID_MAGNETOMETER, "Magnetometer" },
        { BHI385_PHYS_SENSOR_ID_TEMP_GYRO, "Temperature Gyroscope" },
        { BHI385_PHYS_SENSOR_ID_ANY_MOTION, "Any Motion not available now" },
        { BHI385_PHYS_SENSOR_ID_PRESSURE, "Pressure" }, { BHI385_PHYS_SENSOR_ID_POSITION, "Position" },
        { BHI385_PHYS_SENSOR_ID_HUMIDITY, "Humidity" }, { BHI385_PHYS_SENSOR_ID_TEMPERATURE, "Temperature" },
        { BHI385_PHYS_SENSOR_ID_GAS_RESISTOR, "Gas Resistor" },
        { BHI385_PHYS_SENSOR_ID_MAGNETOMETER_DUMMY, "Dummy magnetometer" },
        { BHI385_PHYS_SENSOR_ID_PHYS_STEP_COUNTER, "Step Counter" },
        { BHI385_PHYS_SENSOR_ID_PHYS_STEP_DETECTOR, "Step Detector" },
        { BHI385_PHYS_SENSOR_ID_PHYS_SIGN_MOTION, "Significant Motion" },
        { BHI385_PHYS_SENSOR_ID_PHYS_ANY_MOTION, "Any Motion" },
        { BHI385_PHYS_SENSOR_ID_EX_CAMERA_INPUT, "External Camera Input" }, { BHI385_PHYS_SENSOR_ID_GPS, "GPS" },
        { BHI385_PHYS_SENSOR_ID_LIGHT, "Light" }, { BHI385_PHYS_SENSOR_ID_PROXIMITY, "Proximity" },
        { BHI385_PHYS_SENSOR_ID_ACT_REC, "Activity Recognition" },
        { BHI385_PHYS_SENSOR_ID_PHYS_NO_MOTION, "No Motion" },
        { BHI385_PHYS_SENSOR_ID_WRIST_GESTURE_DETECT, "Wrist Gesture Detector" },
        { BHI385_PHYS_SENSOR_ID_WRIST_WEAR_WAKEUP, "Wrist Wear Wakeup" },
    };

    return bhi385_str_lut_lookup(table, sizeof(table) / sizeof(table[0]), sensor_id, "Undefined sensor ID ");
}

uint8_t get_physical_sensor_id(uint8_t virt_sensor_id)
{
    static const bhi385_u8_lut_entry table[] = {
        { BHI385_SENSOR_ID_ACC_PASS, BHI385_PHYS_SENSOR_ID_ACCELEROMETER },
        { BHI385_SENSOR_ID_ACC_RAW, BHI385_PHYS_SENSOR_ID_ACCELEROMETER },
        { BHI385_SENSOR_ID_ACC, BHI385_PHYS_SENSOR_ID_ACCELEROMETER },
        { BHI385_SENSOR_ID_ACC_BIAS, BHI385_PHYS_SENSOR_ID_ACCELEROMETER },
        { BHI385_SENSOR_ID_ACC_WU, BHI385_PHYS_SENSOR_ID_ACCELEROMETER },
        { BHI385_SENSOR_ID_ACC_RAW_WU, BHI385_PHYS_SENSOR_ID_ACCELEROMETER },
        { BHI385_SENSOR_ID_GYRO_PASS, BHI385_PHYS_SENSOR_ID_GYROSCOPE },
        { BHI385_SENSOR_ID_GYRO_RAW, BHI385_PHYS_SENSOR_ID_GYROSCOPE },
        { BHI385_SENSOR_ID_GYRO, BHI385_PHYS_SENSOR_ID_GYROSCOPE },
        { BHI385_SENSOR_ID_GYRO_BIAS, BHI385_PHYS_SENSOR_ID_GYROSCOPE },
        { BHI385_SENSOR_ID_GYRO_WU, BHI385_PHYS_SENSOR_ID_GYROSCOPE },
        { BHI385_SENSOR_ID_GYRO_RAW_WU, BHI385_PHYS_SENSOR_ID_GYROSCOPE },
        { BHI385_SENSOR_ID_GYRO_BIAS_WU, BHI385_PHYS_SENSOR_ID_GYROSCOPE },
        { BHI385_SENSOR_ID_MAG_PASS, BHI385_PHYS_SENSOR_ID_MAGNETOMETER },
        { BHI385_SENSOR_ID_MAG_RAW, BHI385_PHYS_SENSOR_ID_MAGNETOMETER },
        { BHI385_SENSOR_ID_MAG, BHI385_PHYS_SENSOR_ID_MAGNETOMETER },
        { BHI385_SENSOR_ID_MAG_BIAS, BHI385_PHYS_SENSOR_ID_MAGNETOMETER },
        { BHI385_SENSOR_ID_MAG_WU, BHI385_PHYS_SENSOR_ID_MAGNETOMETER },
        { BHI385_SENSOR_ID_MAG_RAW_WU, BHI385_PHYS_SENSOR_ID_MAGNETOMETER },
        { BHI385_SENSOR_ID_MAG_BIAS_WU, BHI385_PHYS_SENSOR_ID_MAGNETOMETER },
    };

    return bhi385_u8_lut_lookup(table,
                                sizeof(table) / sizeof(table[0]),
                                virt_sensor_id,
                                BHI385_PHYS_SENSOR_ID_NOT_SUPPORTED);
}

char *get_sensor_name(uint8_t sensor_id)
{
    static const bhi385_str_lut_entry table[] = {
        { BHI385_SENSOR_ID_ACC_PASS, "Accelerometer passthrough" },
        { BHI385_SENSOR_ID_ACC_RAW, "Accelerometer uncalibrated" }, { BHI385_SENSOR_ID_ACC, "Accelerometer corrected" },
        { BHI385_SENSOR_ID_ACC_BIAS, "Accelerometer offset" },
        { BHI385_SENSOR_ID_ACC_WU, "Accelerometer corrected wake up" },
        { BHI385_SENSOR_ID_ACC_RAW_WU, "Accelerometer uncalibrated wake up" },
        { BHI385_SENSOR_ID_GYRO_PASS, "Gyroscope passthrough" },
        { BHI385_SENSOR_ID_GYRO_RAW, "Gyroscope uncalibrated" }, { BHI385_SENSOR_ID_GYRO, "Gyroscope corrected" },
        { BHI385_SENSOR_ID_GYRO_BIAS, "Gyroscope offset" }, { BHI385_SENSOR_ID_GYRO_WU, "Gyroscope wake up" },
        { BHI385_SENSOR_ID_GYRO_RAW_WU, "Gyroscope uncalibrated wake up" },
        { BHI385_SENSOR_ID_MAG_PASS, "Magnetometer passthrough" },
        { BHI385_SENSOR_ID_MAG_RAW, "Magnetometer uncalibrated" }, { BHI385_SENSOR_ID_MAG, "Magnetometer corrected" },
        { BHI385_SENSOR_ID_MAG_BIAS, "Magnetometer offset" }, { BHI385_SENSOR_ID_MAG_WU, "Magnetometer wake up" },
        { BHI385_SENSOR_ID_MAG_RAW_WU, "Magnetometer uncalibrated wake up" },
        { BHI385_SENSOR_ID_GRA, "Gravity vector" }, { BHI385_SENSOR_ID_GRA_WU, "Gravity vector wake up" },
        { BHI385_SENSOR_ID_LACC, "Linear acceleration" }, { BHI385_SENSOR_ID_LACC_WU, "Linear acceleration wake up" },
        { BHI385_SENSOR_ID_RV, "Rotation vector" }, { BHI385_SENSOR_ID_RV_WU, "Rotation vector wake up" },
        { BHI385_SENSOR_ID_GAMERV, "Game rotation vector" },
        { BHI385_SENSOR_ID_GAMERV_WU, "Game rotation vector wake up" },
        { BHI385_SENSOR_ID_GEORV, "Geo-magnetic rotation vector" },
        { BHI385_SENSOR_ID_GEORV_WU, "Geo-magnetic rotation vector wake up" }, { BHI385_SENSOR_ID_ORI, "Orientation" },
        { BHI385_SENSOR_ID_ORI_WU, "Orientation wake up" },
        { BHI385_SENSOR_ID_ACC_BIAS_WU, "Accelerometer offset wake up" },
        { BHI385_SENSOR_ID_GYRO_BIAS_WU, "Gyroscope offset wake up" },
        { BHI385_SENSOR_ID_MAG_BIAS_WU, "Magnetometer offset wake up" }, { BHI385_SENSOR_ID_TEMP, "Temperature" },
        { BHI385_SENSOR_ID_BARO, "Barometer" }, { BHI385_SENSOR_ID_HUM, "Humidity" }, { BHI385_SENSOR_ID_GAS, "Gas" },
        { BHI385_SENSOR_ID_TEMP_WU, "Temperature wake up" }, { BHI385_SENSOR_ID_BARO_WU, "Barometer wake up" },
        { BHI385_SENSOR_ID_HUM_WU, "Humidity wake up" }, { BHI385_SENSOR_ID_GAS_WU, "Gas wake up" },
        { BHI385_SENSOR_ID_KLIO, "Klio" }, { BHI385_SENSOR_ID_KLIO_LOG, "Klio log" },
        { BHI385_SENSOR_ID_SI_ACCEL, "SI Accel" }, { BHI385_SENSOR_ID_SI_GYROS, "SI Gyro" },
        { BHI385_SENSOR_ID_LIGHT, "Light" }, { BHI385_SENSOR_ID_LIGHT_WU, "Light wake up" },
        { BHI385_SENSOR_ID_PROX, "Proximity" }, { BHI385_SENSOR_ID_PROX_WU, "Proximity wake up" },
        { BHI385_SENSOR_ID_STC, "Step counter" }, { BHI385_SENSOR_ID_STC_WU, "Step counter wake up" },
        { BHI385_SENSOR_ID_STC_LP, "Low Power Step counter" },
        { BHI385_SENSOR_ID_STC_LP_WU, "Low Power Step counter wake up" },
        { BHI385_SENSOR_ID_SIG, "Significant motion" }, { BHI385_SENSOR_ID_STD, "Step detector" },
        { BHI385_SENSOR_ID_STD_WU, "Step detector wake up" }, { BHI385_SENSOR_ID_TILT_DETECTOR, "Tilt detector" },
        { BHI385_SENSOR_ID_WAKE_GESTURE, "Wake gesture" }, { BHI385_SENSOR_ID_GLANCE_GESTURE, "Glance gesture" },
        { BHI385_SENSOR_ID_PICKUP_GESTURE, "Pickup gesture" }, { BHI385_SENSOR_BMP_TEMPERATURE, "BMP Temperature" },
        { BHI385_SENSOR_ID_SIG_LP_WU, "Low Power Significant motion wake up" },
        { BHI385_SENSOR_ID_STD_LP, "Low Power Step detector" },
        { BHI385_SENSOR_ID_STD_LP_WU, "Low Power Step detector wake up" },
        { BHI385_SENSOR_ID_AR, "Activity recognition" }, { BHI385_SENSOR_ID_EXCAMERA, "External camera trigger" },
        { BHI385_SENSOR_ID_GPS, "GPS" }, { BHI385_SENSOR_ID_WRIST_TILT_GESTURE, "Wrist tilt gesture" },
        { BHI385_SENSOR_ID_DEVICE_ORI, "Device orientation" },
        { BHI385_SENSOR_ID_DEVICE_ORI_WU, "Device orientation wake up" },
        { BHI385_SENSOR_ID_STATIONARY_DET, "Stationary detect" },
        { BHI385_SENSOR_BMP_TEMPERATURE_WU, "BMP Temperature wake up" },
        { BHI385_SENSOR_ID_ANY_MOTION_LP_WU, "Low Power Any motion wake up" },
        { BHI385_SENSOR_ID_NO_MOTION_LP_WU, "Low Power No Motion wake up" },
        { BHI385_SENSOR_ID_MOTION_DET, "Motion detect" },
        { BHI385_SENSOR_ID_AR_WEAR_WU, "Activity recognition for Wearables" },
        { BHI385_SENSOR_ID_WRIST_WEAR_LP_WU, "Low Power Wrist Wear wake up" },
        { BHI385_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, "Low Power Wrist Gesture wake up" },
        { BHI385_SENSOR_ID_MULTI_TAP, "Multi Tap Detector" }, { BHI385_SENSOR_ID_AIR_QUALITY, "Air Quality" },
        { BHI385_SENSOR_ID_HEAD_ORI_MIS_ALG, "Head Misalignment Calibrator" },
        { BHI385_SENSOR_ID_IMU_HEAD_ORI_Q, "IMU Head Orientation Quaternion" },
        { BHI385_SENSOR_ID_NDOF_HEAD_ORI_Q, "NDOF Head Orientation Quaternion" },
        { BHI385_SENSOR_ID_IMU_HEAD_ORI_E, "IMU Head Orientation Euler" },
        { BHI385_SENSOR_ID_NDOF_HEAD_ORI_E, "NDOF Head Orientation Euler" },
        { BHI385_SENSOR_ID_PRESSURE, "BMP Pressure" }, { BHI385_SENSOR_ID_PRESSURE_WU, "BMP Pressure Wakeup" },
    };
    char *ret;

    ret = bhi385_str_lut_lookup(table, sizeof(table) / sizeof(table[0]), sensor_id, NULL);
    if (ret == NULL)
    {
        if ((sensor_id >= BHI385_SENSOR_ID_CUSTOM_START) && (sensor_id <= BHI385_SENSOR_ID_CUSTOM_END))
        {
            ret = "Custom sensor ID ";
        }
        else
        {
            ret = "Undefined sensor ID ";
        }
    }

    return ret;
}

float get_sensor_dynamic_range_scaling(uint8_t sensor_id, float dynamic_range)
{
    static const uint8_t scalable_ids[] = {
        BHI385_SENSOR_ID_ACC_PASS, BHI385_SENSOR_ID_ACC_RAW, BHI385_SENSOR_ID_ACC, BHI385_SENSOR_ID_ACC_BIAS,
        BHI385_SENSOR_ID_ACC_WU, BHI385_SENSOR_ID_ACC_RAW_WU, BHI385_SENSOR_ID_GYRO_PASS, BHI385_SENSOR_ID_GYRO_RAW,
        BHI385_SENSOR_ID_GYRO, BHI385_SENSOR_ID_GYRO_BIAS, BHI385_SENSOR_ID_GYRO_WU, BHI385_SENSOR_ID_GYRO_RAW_WU,
        BHI385_SENSOR_ID_GYRO_BIAS_WU, BHI385_SENSOR_ID_MAG_PASS, BHI385_SENSOR_ID_MAG_RAW, BHI385_SENSOR_ID_MAG,
        BHI385_SENSOR_ID_MAG_BIAS, BHI385_SENSOR_ID_MAG_WU, BHI385_SENSOR_ID_MAG_RAW_WU, BHI385_SENSOR_ID_MAG_BIAS_WU
    };
    size_t i;

    for (i = 0; i < sizeof(scalable_ids) / sizeof(scalable_ids[0]); i++)
    {
        if (scalable_ids[i] == sensor_id)
        {
            return dynamic_range / 32768.0f;
        }
    }

    printf("Sensor ID not supported for dynamic range scaling\r\n");

    return -1.0f; /* Do not apply the scaling factor */
}

char *get_sensor_si_unit(uint8_t sensor_id)
{
    static const bhi385_str_lut_entry table[] = {
        { BHI385_SENSOR_ID_ACC_PASS, "Earth g-s" }, { BHI385_SENSOR_ID_ACC_RAW, "Earth g-s" },
        { BHI385_SENSOR_ID_ACC, "Earth g-s" }, { BHI385_SENSOR_ID_ACC_BIAS, "Earth g-s" },
        { BHI385_SENSOR_ID_ACC_WU, "Earth g-s" }, { BHI385_SENSOR_ID_ACC_RAW_WU, "Earth g-s" },
        { BHI385_SENSOR_ID_GYRO_PASS, "degrees/second" }, { BHI385_SENSOR_ID_GYRO_RAW, "degrees/second" },
        { BHI385_SENSOR_ID_GYRO, "degrees/second" }, { BHI385_SENSOR_ID_GYRO_BIAS, "degrees/second" },
        { BHI385_SENSOR_ID_GYRO_WU, "degrees/second" }, { BHI385_SENSOR_ID_GYRO_RAW_WU, "degrees/second" },
        { BHI385_SENSOR_ID_GYRO_BIAS_WU, "degrees/second" }, { BHI385_SENSOR_ID_MAG_PASS, "microtesla" },
        { BHI385_SENSOR_ID_MAG_RAW, "microtesla" }, { BHI385_SENSOR_ID_MAG, "microtesla" },
        { BHI385_SENSOR_ID_MAG_BIAS, "microtesla" }, { BHI385_SENSOR_ID_MAG_WU, "microtesla" },
        { BHI385_SENSOR_ID_MAG_RAW_WU, "microtesla" }, { BHI385_SENSOR_ID_MAG_BIAS_WU, "microtesla" },
    };

    return bhi385_str_lut_lookup(table, sizeof(table) / sizeof(table[0]), sensor_id, "");
}

char *get_sensor_parse_format(uint8_t sensor_id)
{
    static const bhi385_str_lut_entry table[] = {
        { BHI385_SENSOR_ID_ACC_PASS, "s16,s16,s16" }, { BHI385_SENSOR_ID_ACC_RAW, "s16,s16,s16" },
        { BHI385_SENSOR_ID_ACC, "s16,s16,s16" }, { BHI385_SENSOR_ID_ACC_BIAS, "s16,s16,s16" },
        { BHI385_SENSOR_ID_ACC_BIAS_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_ACC_WU, "s16,s16,s16" },
        { BHI385_SENSOR_ID_ACC_RAW_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_GYRO_PASS, "s16,s16,s16" },
        { BHI385_SENSOR_ID_GYRO_RAW, "s16,s16,s16" }, { BHI385_SENSOR_ID_GYRO, "s16,s16,s16" },
        { BHI385_SENSOR_ID_GYRO_BIAS, "s16,s16,s16" }, { BHI385_SENSOR_ID_GYRO_BIAS_WU, "s16,s16,s16" },
        { BHI385_SENSOR_ID_GYRO_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_GYRO_RAW_WU, "s16,s16,s16" },
        { BHI385_SENSOR_ID_MAG_PASS, "s16,s16,s16" }, { BHI385_SENSOR_ID_MAG_RAW, "s16,s16,s16" },
        { BHI385_SENSOR_ID_MAG, "s16,s16,s16" }, { BHI385_SENSOR_ID_MAG_BIAS, "s16,s16,s16" },
        { BHI385_SENSOR_ID_MAG_BIAS_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_MAG_WU, "s16,s16,s16" },
        { BHI385_SENSOR_ID_MAG_RAW_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_GRA, "s16,s16,s16" },
        { BHI385_SENSOR_ID_GRA_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_LACC, "s16,s16,s16" },
        { BHI385_SENSOR_ID_LACC_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_RV, "s16,s16,s16,s16,u16" },
        { BHI385_SENSOR_ID_RV_WU, "s16,s16,s16,s16,u16" }, { BHI385_SENSOR_ID_GAMERV, "s16,s16,s16,s16,u16" },
        { BHI385_SENSOR_ID_GAMERV_WU, "s16,s16,s16,s16,u16" }, { BHI385_SENSOR_ID_GEORV, "s16,s16,s16,s16,u16" },
        { BHI385_SENSOR_ID_GEORV_WU, "s16,s16,s16,s16,u16" }, { BHI385_SENSOR_ID_ORI, "s16,s16,s16" },
        { BHI385_SENSOR_ID_ORI_WU, "s16,s16,s16" }, { BHI385_SENSOR_ID_DEVICE_ORI, "u8" },
        { BHI385_SENSOR_ID_DEVICE_ORI_WU, "u8" }, { BHI385_SENSOR_ID_HUM, "u8" }, { BHI385_SENSOR_ID_HUM_WU, "u8" },
        { BHI385_SENSOR_ID_PROX, "u8" }, { BHI385_SENSOR_ID_PROX_WU, "u8" }, { BHI385_SENSOR_ID_EXCAMERA, "u8" },
        { BHI385_SENSOR_ID_MULTI_TAP, "u8" }, { BHI385_SENSOR_ID_TEMP, "s16" }, { BHI385_SENSOR_ID_TEMP_WU, "s16" },
        { BHI385_SENSOR_BMP_TEMPERATURE, "s16" }, { BHI385_SENSOR_BMP_TEMPERATURE_WU, "s16" },
        { BHI385_SENSOR_ID_BARO, "u24" }, { BHI385_SENSOR_ID_BARO_WU, "u24" }, { BHI385_SENSOR_ID_GAS, "u32" },
        { BHI385_SENSOR_ID_GAS_WU, "u32" }, { BHI385_SENSOR_ID_STC, "u32" }, { BHI385_SENSOR_ID_STC_WU, "u32" },
        { BHI385_SENSOR_ID_STC_LP, "u32" }, { BHI385_SENSOR_ID_STC_LP_WU, "u32" },
        { BHI385_SENSOR_ID_KLIO, "u8,s8,u8,u8,u8,u8,f,f" }, { BHI385_SENSOR_ID_SI_ACCEL, "f,f,f" },
        { BHI385_SENSOR_ID_SI_GYROS, "f,f,f" }, { BHI385_SENSOR_ID_LIGHT, "s16" }, { BHI385_SENSOR_ID_LIGHT_WU, "s16" },
        { BHI385_SENSOR_ID_SIG, "" }, { BHI385_SENSOR_ID_STD, "" }, { BHI385_SENSOR_ID_STD_WU, "" },
        { BHI385_SENSOR_ID_TILT_DETECTOR, "" }, { BHI385_SENSOR_ID_WAKE_GESTURE, "" },
        { BHI385_SENSOR_ID_GLANCE_GESTURE, "" }, { BHI385_SENSOR_ID_PICKUP_GESTURE, "" },
        { BHI385_SENSOR_ID_SIG_LP_WU, "" }, { BHI385_SENSOR_ID_STD_LP, "" }, { BHI385_SENSOR_ID_STD_LP_WU, "" },
        { BHI385_SENSOR_ID_WRIST_TILT_GESTURE, "" }, { BHI385_SENSOR_ID_STATIONARY_DET, "" },
        { BHI385_SENSOR_ID_ANY_MOTION_LP_WU, "" }, { BHI385_SENSOR_ID_NO_MOTION_LP_WU, "" },
        { BHI385_SENSOR_ID_MOTION_DET, "" }, { BHI385_SENSOR_ID_WRIST_WEAR_LP_WU, "" }, { BHI385_SENSOR_ID_AR, "u16" },
        { BHI385_SENSOR_ID_AR_WEAR_WU, "u16" }, { BHI385_SENSOR_ID_GPS, "st" },
        { BHI385_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, "u8" },
        { BHI385_SENSOR_ID_AIR_QUALITY, "f32,f32,f32,f32,f32,f32,f32,u8" },
        { BHI385_SENSOR_ID_HEAD_ORI_MIS_ALG, "s16,s16,s16,s16" },
        { BHI385_SENSOR_ID_IMU_HEAD_ORI_Q, "s16,s16,s16,s16" }, { BHI385_SENSOR_ID_NDOF_HEAD_ORI_Q, "s16,s16,s16,s16" },
        { BHI385_SENSOR_ID_IMU_HEAD_ORI_E, "s16,s16,s16" }, { BHI385_SENSOR_ID_NDOF_HEAD_ORI_E, "s16,s16,s16" },
    };

    return bhi385_str_lut_lookup(table, sizeof(table) / sizeof(table[0]), sensor_id, "");
}

char *get_sensor_axis_names(uint8_t sensor_id)
{
    static const bhi385_str_lut_entry table[] = {
        { BHI385_SENSOR_ID_ACC_PASS, "x,y,z" }, { BHI385_SENSOR_ID_ACC_RAW, "x,y,z" },
        { BHI385_SENSOR_ID_ACC, "x,y,z" }, { BHI385_SENSOR_ID_ACC_BIAS, "x,y,z" },
        { BHI385_SENSOR_ID_ACC_BIAS_WU, "x,y,z" }, { BHI385_SENSOR_ID_ACC_WU, "x,y,z" },
        { BHI385_SENSOR_ID_ACC_RAW_WU, "x,y,z" }, { BHI385_SENSOR_ID_GYRO_PASS, "x,y,z" },
        { BHI385_SENSOR_ID_GYRO_RAW, "x,y,z" }, { BHI385_SENSOR_ID_GYRO, "x,y,z" },
        { BHI385_SENSOR_ID_GYRO_BIAS, "x,y,z" }, { BHI385_SENSOR_ID_GYRO_BIAS_WU, "x,y,z" },
        { BHI385_SENSOR_ID_GYRO_WU, "x,y,z" }, { BHI385_SENSOR_ID_GYRO_RAW_WU, "x,y,z" },
        { BHI385_SENSOR_ID_MAG_PASS, "x,y,z" }, { BHI385_SENSOR_ID_MAG_RAW, "x,y,z" },
        { BHI385_SENSOR_ID_MAG, "x,y,z" }, { BHI385_SENSOR_ID_MAG_BIAS, "x,y,z" },
        { BHI385_SENSOR_ID_MAG_BIAS_WU, "x,y,z" }, { BHI385_SENSOR_ID_MAG_WU, "x,y,z" },
        { BHI385_SENSOR_ID_MAG_RAW_WU, "x,y,z" }, { BHI385_SENSOR_ID_GRA, "x,y,z" },
        { BHI385_SENSOR_ID_GRA_WU, "x,y,z" }, { BHI385_SENSOR_ID_LACC, "x,y,z" }, { BHI385_SENSOR_ID_LACC_WU, "x,y,z" },
        { BHI385_SENSOR_ID_SI_ACCEL, "x,y,z" }, { BHI385_SENSOR_ID_SI_GYROS, "x,y,z" },
        { BHI385_SENSOR_ID_RV, "x,y,z,w,ar" }, { BHI385_SENSOR_ID_RV_WU, "x,y,z,w,ar" },
        { BHI385_SENSOR_ID_GAMERV, "x,y,z,w,ar" }, { BHI385_SENSOR_ID_GAMERV_WU, "x,y,z,w,ar" },
        { BHI385_SENSOR_ID_GEORV, "x,y,z,w,ar" }, { BHI385_SENSOR_ID_GEORV_WU, "x,y,z,w,ar" },
        { BHI385_SENSOR_ID_ORI, "h,p,r" }, { BHI385_SENSOR_ID_ORI_WU, "h,p,r" }, { BHI385_SENSOR_ID_DEVICE_ORI, "o" },
        { BHI385_SENSOR_ID_DEVICE_ORI_WU, "o" }, { BHI385_SENSOR_ID_TEMP, "t" }, { BHI385_SENSOR_ID_TEMP_WU, "t" },
        { BHI385_SENSOR_BMP_TEMPERATURE, "t" }, { BHI385_SENSOR_BMP_TEMPERATURE_WU, "t" },
        { BHI385_SENSOR_ID_BARO, "p" }, { BHI385_SENSOR_ID_BARO_WU, "p" }, { BHI385_SENSOR_ID_HUM, "h" },
        { BHI385_SENSOR_ID_HUM_WU, "h" }, { BHI385_SENSOR_ID_GAS, "g" }, { BHI385_SENSOR_ID_GAS_WU, "g" },
        { BHI385_SENSOR_ID_KLIO, "lin,lid,lpr,lcr,rin,rid,rc,rsc" }, { BHI385_SENSOR_ID_LIGHT, "l" },
        { BHI385_SENSOR_ID_LIGHT_WU, "l" }, { BHI385_SENSOR_ID_PROX, "p" }, { BHI385_SENSOR_ID_PROX_WU, "p" },
        { BHI385_SENSOR_ID_STC, "c" }, { BHI385_SENSOR_ID_STC_WU, "c" }, { BHI385_SENSOR_ID_STC_LP, "c" },
        { BHI385_SENSOR_ID_STC_LP_WU, "c" }, { BHI385_SENSOR_ID_EXCAMERA, "c" }, { BHI385_SENSOR_ID_SIG, "e" },
        { BHI385_SENSOR_ID_STD, "e" }, { BHI385_SENSOR_ID_STD_WU, "e" }, { BHI385_SENSOR_ID_TILT_DETECTOR, "e" },
        { BHI385_SENSOR_ID_WAKE_GESTURE, "e" }, { BHI385_SENSOR_ID_GLANCE_GESTURE, "e" },
        { BHI385_SENSOR_ID_PICKUP_GESTURE, "e" }, { BHI385_SENSOR_ID_SIG_LP_WU, "e" }, { BHI385_SENSOR_ID_STD_LP, "e" },
        { BHI385_SENSOR_ID_STD_LP_WU, "e" }, { BHI385_SENSOR_ID_WRIST_TILT_GESTURE, "e" },
        { BHI385_SENSOR_ID_STATIONARY_DET, "e" }, { BHI385_SENSOR_ID_ANY_MOTION_LP_WU, "e" },
        { BHI385_SENSOR_ID_NO_MOTION_LP_WU, "e" }, { BHI385_SENSOR_ID_MOTION_DET, "e" },
        { BHI385_SENSOR_ID_WRIST_WEAR_LP_WU, "e" }, { BHI385_SENSOR_ID_AR, "a" }, { BHI385_SENSOR_ID_AR_WEAR_WU, "a" },
        { BHI385_SENSOR_ID_GPS, "g" }, { BHI385_SENSOR_ID_WRIST_GEST_DETECT_LP_WU, "wrist_gesture" },
        { BHI385_SENSOR_ID_MULTI_TAP, "taps" }, { BHI385_SENSOR_ID_AIR_QUALITY, "t,h,g,i,si,c,v,a" },
        { BHI385_SENSOR_ID_HEAD_ORI_MIS_ALG, "x,y,z,w" }, { BHI385_SENSOR_ID_IMU_HEAD_ORI_Q, "x,y,z,w" },
        { BHI385_SENSOR_ID_NDOF_HEAD_ORI_Q, "x,y,z,w" }, { BHI385_SENSOR_ID_IMU_HEAD_ORI_E, "h,p,r" },
        { BHI385_SENSOR_ID_NDOF_HEAD_ORI_E, "h,p,r" },
    };

    return bhi385_str_lut_lookup(table, sizeof(table) / sizeof(table[0]), sensor_id, "");
}

char *get_klio_error(bhi385_klio_param_driver_error_state_t error)
{
    char *ret = "";

    switch (error)
    {
        case KLIO_DRIVER_ERROR_NONE:
            break;
        case KLIO_DRIVER_ERROR_INVALID_PARAMETER:
            ret = "[Klio error] Invalid parameter";
            break;
        case KLIO_DRIVER_ERROR_PARAMETER_OUT_OF_RANGE:
            ret = "[Klio error] Parameter out of range";
            break;
        case KLIO_DRIVER_ERROR_INVALID_PATTERN_OPERATION:
            ret = "[Klio error] Invalid pattern operation";
            break;
        case KLIO_DRIVER_ERROR_NOT_IMPLEMENTED:
            ret = "[Klio error] Not implemented";
            break;
        case KLIO_DRIVER_ERROR_BUFSIZE:
            ret = "[Klio error] Buffer size";
            break;
        case KLIO_DRIVER_ERROR_INTERNAL:
            ret = "[Klio error] Internal";
            break;
        case KLIO_DRIVER_ERROR_UNDEFINED:
            ret = "[Klio error] Undefined";
            break;
        case KLIO_DRIVER_ERROR_OPERATION_PENDING:
            ret = "[Klio error] Operation pending";
            break;
        default:
            ret = "[Klio error] Unknown error code";
    }

    return ret;
}

#ifndef PC
void default_verbose_write(uint8_t *buffer, uint16_t length)
{
    coines_write_intf(COINES_COMM_INTF_USB, buffer, length);
}

void verbose_write(uint8_t *buffer, uint16_t length) __attribute__ ((weak, alias("default_verbose_write")));

#endif

void print_api_error(int8_t rslt, struct bhi385_dev *dev)
{
    if (rslt != BHI385_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHI385_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHI385_INTF_RET_SUCCESS;
        }
    }
}

void upload_firmware(const uint8_t fw[], uint32_t length, struct bhi385_dev *dev)
{
    uint8_t sensor_error = 0;
    int8_t temp_rslt;
    int8_t rslt = BHI385_OK;

    printf("Loading firmware into RAM.\r\n");
    rslt = bhi385_upload_firmware_to_ram(fw, length, dev);

    temp_rslt = bhi385_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);

    printf("Booting from RAM.\r\n");
    rslt = bhi385_boot_from_ram(dev);

    temp_rslt = bhi385_get_error_value(&sensor_error, dev);
    if (sensor_error)
    {
        printf("%s\r\n", get_sensor_error_text(sensor_error));
    }

    print_api_error(rslt, dev);
    print_api_error(temp_rslt, dev);
}

void setup_host_int_ctrl(struct bhi385_dev *dev)
{
    int8_t rslt;
    uint8_t hintr_ctrl, hif_ctrl;

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;

    rslt = bhi385_set_host_interrupt_ctrl(hintr_ctrl, dev);
    print_api_error(rslt, dev);
    rslt = bhi385_get_host_interrupt_ctrl(&hintr_ctrl, dev);
    print_api_error(rslt, dev);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHI385_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHI385_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHI385_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi385_set_host_intf_ctrl(hif_ctrl, dev);
    print_api_error(rslt, dev);
}

void init_sensor(struct bhi385_dev *dev, enum bhi385_intf intf)
{
    uint8_t chip_id = 0;
    int8_t rslt;

#ifdef BHI385_USE_I2C
    rslt = bhi385_init(intf, bhi385_i2c_read, bhi385_i2c_write, bhi385_delay_us, BHI385_RD_WR_LEN, NULL, dev);
#else
    rslt = bhi385_init(intf, bhi385_spi_read, bhi385_spi_write, bhi385_delay_us, BHI385_RD_WR_LEN, NULL, dev);
#endif

    print_api_error(rslt, dev);

    rslt = bhi385_soft_reset(dev);
    print_api_error(rslt, dev);

    rslt = bhi385_get_chip_id(&chip_id, dev);
    print_api_error(rslt, dev);

    /* Check for a valid Chip ID */
    if (chip_id == BHI385_CHIP_ID)
    {
        printf("Chip ID read 0x%X\r\n", chip_id);
    }
    else
    {
        printf("Device not found. Chip ID read 0x%X\r\n", chip_id);

        return;
    }
}