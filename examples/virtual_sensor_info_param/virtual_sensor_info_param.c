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
 * @file    virtual_sensor_info_param.c
 * @brief   Example to get virtual sensor information parameters
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bhi385.h"
#include "bhi385_parse.h"
#include "common.h"
#include "bhi385_virtual_sensor_info_param.h"
#include "bhi385_param_defs.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

static void print_api_error(int8_t rslt, struct bhi385_dev *dev);
static int8_t upload_firmware(struct bhi385_dev *dev);

enum bhi385_intf intf;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi385_dev bhy;
    struct bhi385_virtual_sensor_info_param_info info;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;

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

    /* Check the interrupt pin and FIFO configurations. Disable status and debug */
    hintr_ctrl = BHI385_ICTL_DISABLE_STATUS_FIFO | BHI385_ICTL_DISABLE_DEBUG;

    rslt = bhi385_set_host_interrupt_ctrl(hintr_ctrl, &bhy);
    print_api_error(rslt, &bhy);
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

    /* Configure the host interface */
    hif_ctrl = 0;
    rslt = bhi385_set_host_intf_ctrl(hif_ctrl, &bhy);
    print_api_error(rslt, &bhy);

    /* Check if the sensor is ready to load firmware */
    rslt = bhi385_get_boot_status(&boot_status, &bhy);
    print_api_error(rslt, &bhy);

    if (boot_status & BHI385_BST_HOST_INTERFACE_READY)
    {
        uint8_t sensor_error;
        int8_t temp_rslt;
        printf("Loading firmware.\r\n");

        rslt = upload_firmware(&bhy);
        temp_rslt = bhi385_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

        printf("Booting from RAM.\r\n");
        rslt = bhi385_boot_from_ram(&bhy);

        temp_rslt = bhi385_get_error_value(&sensor_error, &bhy);
        if (sensor_error)
        {
            printf("%s\r\n", get_sensor_error_text(sensor_error));
        }

        print_api_error(rslt, &bhy);
        print_api_error(temp_rslt, &bhy);

        rslt = bhi385_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI385_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        printf("Get virtual sensor information.\r\n");
        rslt = bhi385_virtual_sensor_info_param_get_info(BHI385_SENSOR_ID_ACC_PASS, &info, &bhy);
        print_api_error(rslt, &bhy);
        printf("    Sensor ID: %u\r\n", info.sensor_type);
        printf("    Driver ID: %u\r\n", info.driver_id);
        printf("    Driver version: %u\r\n", info.driver_version);
        printf("    Power: %u\r\n", info.power);
        printf("    Max range: %u\r\n", info.max_range.u16_val);
        printf("    Resolution: %u\r\n", info.resolution.u16_val);
        printf("    Max rate: %f\r\n", info.max_rate.f_val);
#ifdef PC
        printf("    FIFO reserved: %u\r\n", info.fifo_reserved.u32_val);
        printf("    FIFO max: %u\r\n", info.fifo_max.u32_val);
#else
        printf("    FIFO reserved: %lu\r\n", info.fifo_reserved.u32_val);
        printf("    FIFO max: %lu\r\n", info.fifo_max.u32_val);
#endif
        printf("    Event size: %u\r\n", info.event_size);
        printf("    Min rate: %f\r\n", info.min_rate.f_val);
    }
    else
    {
        printf("Host interface not ready. Exiting\r\n");

        close_interfaces(intf);

        return 0;
    }

    close_interfaces(intf);

    return rslt;
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

static int8_t upload_firmware(struct bhi385_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhi385_firmware_image);
    int8_t rslt = BHI385_OK;

    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHI385_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

        rslt = bhi385_upload_firmware_to_ram_partly(&bhi385_firmware_image[i], len, i, incr, dev);

        printf("%.2f%% complete\r", (float)(i + incr) / (float)len * 100.0f);
    }

    printf("\n");

    return rslt;
}
