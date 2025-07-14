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
 * @file    soft_passthrough.c
 * @brief   Example for soft pass-through
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhi385.h"
#include "bhi385_event_data.h"
#include "common.h"
#include "bhi385_virtual_sensor_conf_param.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

#define WORK_BUFFER_SIZE                UINT16_C(2048)

/*! BMM350 Macro Definitions */
#define BMM150_I2C_SLAVE_ADDR           UINT8_C(0x10)
#define BMM150_CHIP_ID_REG              UINT8_C(0x40)
#define BMM150_POWER_CONTROL_REG        UINT8_C(0x4B)
#define BMM150_POWER_CONTROL_REG_VALUE  UINT8_C(0x81)
#define BMM150_CHIP_ID                  UINT8_C(0x32)

/*! BME688 Macro Definitions */
#define BME688_I2C_SLAVE_ADDR           UINT8_C(0x77)
#define BME688_CHIP_ID_REG              UINT8_C(0xD0)
#define BME688_RESET_REG                UINT8_C(0xE0)
#define BME688_RESET_REG_VALUE          UINT8_C(0xB6)
#define BME688_CHIP_ID                  UINT8_C(0x61)

/*! BMI270 Macro Definitions */
#define BMI270_ACC_CONF_REG             UINT8_C(0x40)
#define BMI270_ACC_RANGE_16G            UINT8_C(0x02) /*in BHI385, BMI270 updated Acc Range, also the Register Value */

#define CHIP_ID_READ_ON_POR             UINT8_C(0x00)
#define MAX_PASSTHROUGH_TRANSFER_BYTE   UINT8_C(244)
#define SPI_TRANSFER_RATE               UINT16_C(0xE803)
#define SPI_M1_CS_PIN                   UINT8_C(25)
#define SCALING_FACTOR_INVALID_LIMIT    -1.0f

/*! @brief Parse meta event.
 *
 *  @param[in] callback_info : Sensor data info.
 *  @param[in] callback_ref  : Parse reference.
 */
static void parse_meta_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/*! @brief Prints API error code.
 *
 *  @param[in] rslt      : API Error code.
 *  @param[in] dev       : Device reference.
 */
static void print_api_error(int8_t rslt, struct bhi385_dev *dev);

/*! @brief Loads firmware image to BHy ram.
 *
 *  @param[in] boot_stat : Boot status.
 *  @param[in] dev       : Device reference.
 */
static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev);

/*! @brief Soft Pass-Through function to read from register via I2C.
 *
 *  @param[in] sif_bus        : soft-passthrough interface
 *  @param[in] dev_addr   : Device address.
 *  @param[in] reg_addr   : Register address to be read from.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] length     : Length of the data buffer.
 *  @param[in] dev        : Device reference.
 */
static void spt_i2c_read(uint8_t sif_bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t *reg_data,
                         uint8_t length,
                         struct bhi385_dev *dev);

/*! @brief Soft Pass-Through function to write to register via I2C.
 *
 *  @param[in] sif_bus        : soft-passthrough interface
 *  @param[in] dev_addr   : Device address.
 *  @param[in] reg_addr   : Register address to write to.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] length     : Length of the data buffer.
 *  @param[in] dev        : Device reference.
 */
static void spt_i2c_write(uint8_t sif_bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t *reg_data,
                          uint8_t length,
                          struct bhi385_dev *dev);

/*! @brief Parse accel data.
 *
 *  @param[in] callback_info : sensor data info.
 *  @param[in] callback_ref  : Parse reference.
 */
static void parse_accel(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/*! @brief Soft Pass-Through function to read from register via SPI.
 *
 *  @param[in] dev        : Device reference.
 *  @param[in] reg_addr   : Register address.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] read_cnt   : Length of the data to be read.
 */
static void spt_spi0_read(uint8_t reg_addr, uint8_t* reg_data, uint8_t read_cnt, struct bhi385_dev *dev);

/*! @brief Soft Pass-Through function to write to register via SPI.
 *
 *  @param[in] dev        : Device reference.
 *  @param[in] reg_addr   : Register address to write to.
 *  @param[in] reg_data   : Reference to the data buffer.
 *  @param[in] write_cnt  : Length of the data to be written.
 */
static void spt_spi0_write(uint8_t reg_addr, uint8_t* reg_data, uint8_t write_cnt, struct bhi385_dev *dev);

enum bhi385_intf intf;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi385_dev bhy;
    uint8_t reg_data[16];
    uint8_t hif_ctrl, boot_status, hintr_ctrl;
    uint8_t data_length = 0;
    uint8_t write_data[16] = { 0 };
    uint8_t accuracy; /* Accuracy is reported as a meta event. It is being printed alongside the data */
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint8_t loop = 0;
    uint8_t limit = 50;

#ifdef BHI385_USE_I2C
    intf = BHI385_I2C_INTERFACE;
#else
    intf = BHI385_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHI385_USE_I2C
    rslt = bhi385_init(BHI385_I2C_INTERFACE,
                       bhi385_i2c_read,
                       bhi385_i2c_write,
                       bhi385_delay_us,
                       BHI385_RD_WR_LEN,
                       NULL,
                       &bhy);
#else
    rslt = bhi385_init(BHI385_SPI_INTERFACE,
                       bhi385_spi_read,
                       bhi385_spi_write,
                       bhi385_delay_us,
                       BHI385_RD_WR_LEN,
                       NULL,
                       &bhy);
#endif
    print_api_error(rslt, &bhy);

    rslt = bhi385_soft_reset(&bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_get_chip_id(&chip_id, &bhy);
    print_api_error(rslt, &bhy);

    /* Check for a valid Chip ID */
    if (chip_id == BHI385_CHIP_ID)
    {
        printf("Chip ID read 0x%X\r\n", chip_id);
    }
    else
    {
        printf("Device not found. Chip ID read 0x%X\r\n", chip_id);
    }

    /* Configure the host interface */
    hif_ctrl = BHI385_HIF_CTRL_ASYNC_STATUS_CHANNEL;
    rslt = bhi385_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;

    rslt = bhi385_get_host_interrupt_ctrl(&hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    printf("Host interrupt control\r\n");
    printf("    Wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
    printf("    Non wake up FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
    printf("    Status FIFO %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
    printf("    Debugging %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
    printf("    Fault %s.\r\n", (hintr_ctrl & BHI385_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
    printf("    Interrupt is %s.\r\n", (hintr_ctrl & BHI385_ICTL_ACTIVE_LOW) ? "active low" : "active high");
    printf("    Interrupt is %s triggered.\r\n", (hintr_ctrl & BHI385_ICTL_EDGE) ? "pulse" : "level");
    printf("    Interrupt pin drive is %s.\r\n", (hintr_ctrl & BHI385_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

    /* Check if the sensor is ready to load firmware */
    rslt = bhi385_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI385_BST_HOST_INTERFACE_READY)
    {
        upload_firmware(boot_status, &bhy);

        rslt = bhi385_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI385_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt =
            bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU, parse_meta_event, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_ACC_WU, parse_accel, (void*)&accuracy, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
        print_api_error(rslt, &bhy);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    /* Attempt to read, M1 SPI interface to access BMI270 */
    printf("BMI270 Accelerometer Configuration Register Read\r\n");
    data_length = 8;
    spt_spi0_read(BMI270_ACC_CONF_REG, reg_data, data_length, &bhy);

    data_length = 2;
    write_data[0] = reg_data[1];
    write_data[1] = BMI270_ACC_RANGE_16G; /* set accelerometer g-range to 16g */
    spt_spi0_write(BMI270_ACC_CONF_REG, write_data, data_length, &bhy);

    data_length = 8;
    spt_spi0_read(BMI270_ACC_CONF_REG, reg_data, data_length, &bhy);

    if (reg_data[2] == BMI270_ACC_RANGE_16G)
    {
        printf("BMI270 Accelerometer g-range set at 16G\r\n");
    }

    /* Clear the buffer */
    memset(reg_data, 0, sizeof(reg_data));
    memset(write_data, 0, sizeof(write_data));

    /* Attempt to read out the chip ID from the BMM150 connected over SIF2 of the BHI385 - on power on reset */
    data_length = 1;
    spt_i2c_read(BHI385_SPASS_SIF2, BMM150_I2C_SLAVE_ADDR, BMM150_CHIP_ID_REG, reg_data, data_length, &bhy);
    if (reg_data[0] == CHIP_ID_READ_ON_POR)
    {
        printf("BMM150 Chip ID read returns 0x00: In suspend mode\r\n"); /* Set the power bit to 1 in power control
                                                                          * register (0x4B), to put the device from
                                                                          * suspend mode to sleep mode. In this state
                                                                          * Chip ID can be read */
    }

    /* Attempt to set the power bit and soft reset trigger bit to 1 in power control register (0x4B); bit7： Soft reset,
     * bit0: power control bit */
    write_data[0] = BMM150_POWER_CONTROL_REG_VALUE;
    data_length = 1;
    printf("BMM150 Power Control Register Write\r\n");
    spt_i2c_write(BHI385_SPASS_SIF2, BMM150_I2C_SLAVE_ADDR, BMM150_POWER_CONTROL_REG, write_data, data_length, &bhy);

    /* Attempt to read out the BMM150 chip ID after writing power control register */
    spt_i2c_read(BHI385_SPASS_SIF2, BMM150_I2C_SLAVE_ADDR, BMM150_CHIP_ID_REG, reg_data, data_length, &bhy);
    if (reg_data[0] == BMM150_CHIP_ID)
    {
        printf("BMM150 Chip ID read returns: %X In sleep mode\r\n", reg_data[0]);
    }

    /* Attempt to reset BME688 from unknown status, connected over SIF3 of the BHI385 */
    printf("BME688 Reset Register Write\r\n");
    write_data[0] = BME688_RESET_REG_VALUE;
    data_length = 1;
    spt_i2c_write(BHI385_SPASS_SIF3, BME688_I2C_SLAVE_ADDR, BME688_RESET_REG, write_data, data_length, &bhy);

    /* Attempt to read out BME688 chip ID after reset */
    spt_i2c_read(BHI385_SPASS_SIF3, BME688_I2C_SLAVE_ADDR, BME688_CHIP_ID_REG, reg_data, data_length, &bhy);
    if (reg_data[0] == BME688_CHIP_ID)
    {
        printf("BME688 Chip ID read returns: %X\r\n", reg_data[0]);
    }

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi385_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    sensor_conf.sample_rate = 100.0f; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_ACC_WU, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_ACC_WU), sensor_conf.sample_rate);
    printf("Data Acquisition:\r\n");

    while (rslt == BHI385_OK && loop < limit)
    {
        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            loop++;
            print_api_error(rslt, &bhy);
        }
    }

    close_interfaces(intf);

    return rslt;
}

static void parse_accel(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi385_event_data_xyz data;
    float scaling_factor = SCALING_FACTOR_INVALID_LIMIT;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    bhi385_event_data_parse_xyz(callback_info->data_ptr, &data);

    scaling_factor = (float)BHI385_ACCEL_16G / 32768.0f; /* since the sensor data will be scaled to signed 16bits */

    printf(" x: %f, y: %f, z: %f; unit: LSB/G\r\n",
           data.x * scaling_factor,
           data.y * scaling_factor,
           data.z * scaling_factor);

}

static void spt_spi0_read(uint8_t reg_addr, uint8_t* reg_data, uint8_t read_cnt, struct bhi385_dev *dev)
{
    uint8_t rslt = BHI385_OK;
    uint8_t transfer_bytes = 0;
    union bhi385_soft_passthrough_conf conf;

    /* transfer bytes */
    if (read_cnt > MAX_PASSTHROUGH_TRANSFER_BYTE)
    {
        transfer_bytes = MAX_PASSTHROUGH_TRANSFER_BYTE;
    }
    else
    {
        transfer_bytes = read_cnt;
    }

    conf.conf.direction = BHI385_SPASS_READ;
    conf.conf.trans_type = BHI385_SPASS_SINGLE_TRANS;
    conf.conf.delay_ctrl = BHI385_SPASS_DELAY_DIS;
    conf.conf.master_bus = BHI385_SPASS_SIF1;

    conf.conf.spi_mode = BHI385_SPASS_SPI_4_WIRE;
    conf.conf.cpol = BHI385_SPASS_SPI_CPOL_1;
    conf.conf.cpha = BHI385_SPASS_SPI_CPHA_1;
    conf.conf.cs_level = BHI385_SPASS_SPI_CS_LOW;
    conf.conf.lsb_first = BHI385_SPASS_SPI_LSB_FIRST_DIS;
    conf.conf.trans_rate = SPI_TRANSFER_RATE; /*in kHz, 16-bit unsigned, LSB first 0x03E8 = 1MHz */
    conf.conf.address_shift = 0;
    conf.conf.read_bit_pol = 1;
    conf.conf.read_bit_pos = BHI385_SPASS_SPI_READ_BIT_POS_7;
    conf.conf.func_set.cs_pin = SPI_M1_CS_PIN;

    conf.conf.trans_count = transfer_bytes + 1;
    conf.conf.reg = reg_addr;

    /* BMI270 Reg_data[0] is dummy value, reg_data[1] is register value */
    rslt = bhi385_soft_passthrough_transfer(&conf, conf.conf.reg, conf.conf.trans_count, reg_data, dev);

    print_api_error(rslt, dev);
}

static void spt_spi0_write(uint8_t reg_addr, uint8_t* reg_data, uint8_t write_cnt, struct bhi385_dev *dev)
{
    uint8_t rslt = BHI385_OK;
    uint8_t transfer_bytes = 0;
    union bhi385_soft_passthrough_conf conf;

    /* transfer bytes */
    if (write_cnt > MAX_PASSTHROUGH_TRANSFER_BYTE)
    {
        transfer_bytes = MAX_PASSTHROUGH_TRANSFER_BYTE;
    }
    else
    {
        transfer_bytes = write_cnt;
    }

    conf.conf.direction = BHI385_SPASS_WRITE;
    conf.conf.trans_type = BHI385_SPASS_MULTI_TRANS; /* multiple bytes to write */
    conf.conf.delay_ctrl = BHI385_SPASS_DELAY_DIS;
    conf.conf.master_bus = BHI385_SPASS_SIF1;

    conf.conf.spi_mode = BHI385_SPASS_SPI_4_WIRE;
    conf.conf.cpol = BHI385_SPASS_SPI_CPOL_1;
    conf.conf.cpha = BHI385_SPASS_SPI_CPHA_1;
    conf.conf.cs_level = BHI385_SPASS_SPI_CS_LOW;
    conf.conf.lsb_first = BHI385_SPASS_SPI_LSB_FIRST_DIS;
    conf.conf.trans_rate = SPI_TRANSFER_RATE; /*in kHz, 16-bit unsigned, LSB first 0x03E8 = 1MHz */
    conf.conf.address_shift = 0;
    conf.conf.read_bit_pol = 1;
    conf.conf.read_bit_pos = BHI385_SPASS_SPI_READ_BIT_POS_7;
    conf.conf.func_set.cs_pin = SPI_M1_CS_PIN;

    conf.conf.trans_count = transfer_bytes;
    conf.conf.reg = reg_addr;

    rslt = bhi385_soft_passthrough_transfer(&conf, conf.conf.reg, conf.conf.trans_count, reg_data, dev);
    print_api_error(rslt, dev);
}

static void spt_i2c_read(uint8_t sif_bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t *reg_data,
                         uint8_t length,
                         struct bhi385_dev *dev)
{
    int8_t rslt = BHI385_OK;
    union bhi385_soft_passthrough_conf spt_conf = { { 0 } };

    spt_conf.conf.direction = BHI385_SPASS_READ;
    spt_conf.conf.trans_type = BHI385_SPASS_SINGLE_TRANS;
    spt_conf.conf.master_bus = sif_bus;
    spt_conf.conf.func_set.slave_address = dev_addr;

    rslt = bhi385_soft_passthrough_transfer(&spt_conf, reg_addr, length, reg_data, dev);
    print_api_error(rslt, dev);
}

static void spt_i2c_write(uint8_t sif_bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t *reg_data,
                          uint8_t length,
                          struct bhi385_dev *dev)
{
    int8_t rslt = BHI385_OK;
    union bhi385_soft_passthrough_conf spt_conf = { { 0 } };

    spt_conf.conf.direction = BHI385_SPASS_WRITE;
    spt_conf.conf.trans_type = BHI385_SPASS_SINGLE_TRANS;
    spt_conf.conf.master_bus = sif_bus;
    spt_conf.conf.func_set.slave_address = dev_addr;

    rslt = bhi385_soft_passthrough_transfer(&spt_conf, reg_addr, length, reg_data, dev);
    print_api_error(rslt, dev);
}

static void parse_meta_event(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    uint8_t *accuracy = (uint8_t*)callback_ref;
    char *event_text;

    if (callback_info->sensor_id == BHI385_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHI385_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    switch (meta_event_type)
    {
        case BHI385_META_EVENT_FLUSH_COMPLETE:
            printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_SAMPLE_RATE_CHANGED:
            printf("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_POWER_MODE_CHANGED:
            printf("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_ALGORITHM_EVENTS:
            printf("%s Algorithm event\r\n", event_text);
            break;
        case BHI385_META_EVENT_SENSOR_STATUS:
            printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            if (accuracy)
            {
                *accuracy = byte2;
            }

            break;
        case BHI385_META_EVENT_BSX_DO_STEPS_MAIN:
            printf("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHI385_META_EVENT_BSX_DO_STEPS_CALIB:
            printf("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHI385_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            printf("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHI385_META_EVENT_SENSOR_ERROR:
            printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHI385_META_EVENT_FIFO_OVERFLOW:
            printf("%s FIFO overflow\r\n", event_text);
            break;
        case BHI385_META_EVENT_DYNAMIC_RANGE_CHANGED:
            printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_FIFO_WATERMARK:
            printf("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHI385_META_EVENT_INITIALIZED:
            printf("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHI385_META_TRANSFER_CAUSE:
            printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_SENSOR_FRAMEWORK:
            printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHI385_META_EVENT_RESET:
            printf("%s Reset event\r\n", event_text);
            break;
        case BHI385_META_EVENT_SPACER:
            break;
        default:
            printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static void print_api_error(int8_t rslt, struct bhi385_dev *dev)
{
    if (rslt != BHI385_OK)
    {
        printf("%s\r\n", get_api_error(rslt));
        if ((rslt == BHI385_E_IO) && (dev != NULL))
        {
            printf("%s\r\n", get_coines_error(dev->hif.intf_rslt));
            dev->hif.intf_rslt = BHI385_INTF_RET_SUCCESS;
        }

        exit(0);
    }
}

static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev)
{
    uint8_t sensor_error;
    int8_t temp_rslt;
    int8_t rslt = BHI385_OK;

    printf("Loading firmware into RAM.\r\n");
    rslt = bhi385_upload_firmware_to_ram(bhi385_firmware_image, sizeof(bhi385_firmware_image), dev);
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
