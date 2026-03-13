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
 * @file    dynamic_range.c
 * @brief   Example to change sensor dynamic range
 *
 */
#include <stdio.h>
#include "common.h"
#include "bhi385_parse.h"
#include "bhi385/Bosch_Shuttle3_BHI385_BMM350_BME688_bsxsam_ndof.fw.h"

#define SCALING_FACTOR_INVALID_LIMIT  -1.0f

#define PHYSICAL_ACCEL_ID             1
#define PHYSICAL_GYRO_ID              3
#define PHYSICAL_MAG_ID               5

/*! @brief Parse gyro data.
 *
 *  @param[in] callback_info : sensor data info.
 *  @param[in] callback_ref  : Parse reference.
 */
static void parse_3axis(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);
static float get_sensor_scaling(uint8_t sensor_id, uint16_t dynamic_range[]);

enum bhi385_intf intf;
struct bhi385_dev bhy;
uint16_t range[8] = { 0 };

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t hif_ctrl, boot_status, hintr_ctrl;
    uint8_t accuracy; /* Accuracy is reported as a meta event. It is being printed alongside the data */
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t loop = 0;
    uint8_t limit = 30;

#ifdef BHI385_USE_I2C
    intf = BHI385_I2C_INTERFACE;
#else
    intf = BHI385_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

#ifdef BHI385_USE_I2C
    rslt = bhi385_init(intf, bhi385_i2c_read, bhi385_i2c_write, bhi385_delay_us, BHI385_RD_WR_LEN, NULL, &bhy);
#else
    rslt = bhi385_init(intf, bhi385_spi_read, bhi385_spi_write, bhi385_delay_us, BHI385_RD_WR_LEN, NULL, &bhy);
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
        upload_firmware(bhi385_firmware_image, sizeof(bhi385_firmware_image), &bhy);

        rslt = bhi385_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI385_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT,
                                                   bhi385_parse_meta_event,
                                                   (void*)&accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU,
                                                   bhi385_parse_meta_event,
                                                   (void*)&accuracy,
                                                   &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_ACC_PASS, parse_3axis, NULL, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_ACC_RAW, parse_3axis, NULL, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_GYRO_RAW, parse_3axis, NULL, &bhy);
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

    /* Update the callback table to enable parsing of sensor data */
    rslt = bhi385_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    struct bhi385_system_param_phys_sensor_info phy_acc_info, phy_gyro_info, phy_mag_info;

    /* Get the default range of accel and gyro */
    rslt = bhi385_system_param_get_physical_sensor_info(PHYSICAL_ACCEL_ID, &phy_acc_info, &bhy);
    print_api_error(rslt, &bhy);
    printf("Accel Range is %u\r\n", phy_acc_info.curr_range.u16_val);

    rslt = bhi385_system_param_get_physical_sensor_info(PHYSICAL_GYRO_ID, &phy_gyro_info, &bhy);
    print_api_error(rslt, &bhy);
    printf("Gyro Range is %u\r\n", phy_gyro_info.curr_range.u16_val);
    range[PHYSICAL_GYRO_ID] = phy_gyro_info.curr_range.u16_val;

    /* Get the mag dynamic range which doesn't support to change */
    rslt = bhi385_system_param_get_physical_sensor_info(PHYSICAL_MAG_ID, &phy_mag_info, &bhy);
    print_api_error(rslt, &bhy);
    printf("Mag Range is %u\r\n", phy_mag_info.curr_range.u16_val);
    range[PHYSICAL_MAG_ID] = phy_mag_info.curr_range.u16_val;

    /* Set the sensor dynamic range
       1. The sensor ID must correspond to a specific physical sensor’s virtual sensor (either wake-up or non-wake-up).
          It must also be for a physical sensor supported by the currently loaded firmware image.
          Setting the dynamic range is not supported for any virtual sensor being derived from multiple physical sensors,
          e.g. Rotation Vector.
       2. If the same physical sensor range is configured via multiple virtual sensors, the highest dynamic range value
          will be used. Lower range settings are not effective until the virtual sensor’s highest range is reduced.
    */
    rslt = bhi385_set_virt_sensor_range(BHI385_SENSOR_ID_ACC_PASS, BHI385_ACCEL_4G, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_set_virt_sensor_range(BHI385_SENSOR_ID_ACC, BHI385_ACCEL_16G, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_set_virt_sensor_range(BHI385_SENSOR_ID_ACC_RAW, BHI385_ACCEL_8G, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_system_param_get_physical_sensor_info(PHYSICAL_ACCEL_ID, &phy_acc_info, &bhy);
    print_api_error(rslt, &bhy);
    printf("Accel Range is %u\r\n", phy_acc_info.curr_range.u16_val);
    range[PHYSICAL_ACCEL_ID] = phy_acc_info.curr_range.u16_val;

    sensor_conf.sample_rate = 10.0f; /* Read out data measured at 10Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_ACC_PASS, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_ACC_PASS), sensor_conf.sample_rate);

    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_GYRO_RAW, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_GYRO_RAW), sensor_conf.sample_rate);

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

static void parse_3axis(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi385_event_data_xyz data;
    float scaling_factor = SCALING_FACTOR_INVALID_LIMIT;
    uint32_t s, ns;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */
    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

    bhi385_event_data_parse_xyz(callback_info->data_ptr, &data);

    scaling_factor = get_sensor_scaling(callback_info->sensor_id, range);
    if (scaling_factor > SCALING_FACTOR_INVALID_LIMIT)
    {
#ifndef PC

        printf("SID: %u; T: %lu .%09lu; x: %f, y: %f, z: %f;\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.x * scaling_factor,
               data.y * scaling_factor,
               data.z * scaling_factor);
#else
        printf("SID: %u; T: %u .%09u; x: %f, y: %f, z: %f;\r\n",
               callback_info->sensor_id,
               s,
               ns,
               data.x * scaling_factor,
               data.y * scaling_factor,
               data.z * scaling_factor);
#endif
    }
}

static float get_sensor_scaling(uint8_t sensor_id, uint16_t dynamic_range[])
{
    float scaling = -1.0f;

    switch (sensor_id)
    {
        case BHI385_SENSOR_ID_ACC_PASS:
        case BHI385_SENSOR_ID_ACC_RAW:
        case BHI385_SENSOR_ID_ACC:
        case BHI385_SENSOR_ID_ACC_BIAS:
        case BHI385_SENSOR_ID_ACC_WU:
        case BHI385_SENSOR_ID_ACC_RAW_WU:
            scaling = dynamic_range[PHYSICAL_ACCEL_ID] / 32768.0f;
            break;
        case BHI385_SENSOR_ID_GYRO_PASS:
        case BHI385_SENSOR_ID_GYRO_RAW:
        case BHI385_SENSOR_ID_GYRO:
        case BHI385_SENSOR_ID_GYRO_BIAS:
        case BHI385_SENSOR_ID_GYRO_WU:
        case BHI385_SENSOR_ID_GYRO_RAW_WU:
        case BHI385_SENSOR_ID_GYRO_BIAS_WU:
            scaling = dynamic_range[PHYSICAL_GYRO_ID] / 32768.0f;
            break;
        case BHI385_SENSOR_ID_MAG_PASS:
        case BHI385_SENSOR_ID_MAG_RAW:
        case BHI385_SENSOR_ID_MAG:
        case BHI385_SENSOR_ID_MAG_BIAS:
        case BHI385_SENSOR_ID_MAG_WU:
        case BHI385_SENSOR_ID_MAG_RAW_WU:
        case BHI385_SENSOR_ID_MAG_BIAS_WU:
            scaling = dynamic_range[PHYSICAL_MAG_ID] / 32768.0f;
            break;
        default:
            printf("Sensor ID not supported for dynamic range scaling\r\n");
            scaling = -1.0f; /* Do not apply the scaling factor */
    }

    return scaling;
}