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
 * @file    bsx_algo_param.c
 * @brief   Bsx algorithm configuration example for the BHY
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "bhi385.h"
#include "bhi385_parse.h"
#include "common.h"
#include "bhi385_bsx_algo_param.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

static void print_api_error(int8_t rslt, struct bhi385_dev *dev);
static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev);

#define PARAMETER_ID  0X201

enum bhi385_intf intf;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi385_dev bhy;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    struct bhi385_bsx_algo_param_version get_bsx_ver = { 0 };

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
        upload_firmware(boot_status, &bhy);

        rslt = bhi385_get_kernel_version(&version, &bhy);
        print_api_error(rslt, &bhy);
        if ((rslt == BHI385_OK) && (version != 0))
        {
            printf("Boot successful. Kernel version %u.\r\n", version);
        }

        /* get the BSX version */
        rslt = bhi385_bsx_algo_param_get_bsx_version(&get_bsx_ver, &bhy);
        print_api_error(rslt, &bhy);

        printf("\n");
        printf("BSX version : %u.%u.%u.%u\r\n",
               get_bsx_ver.major_version,
               get_bsx_ver.minor_version,
               get_bsx_ver.major_bug_fix_version,
               get_bsx_ver.minor_bug_fix_version);

        /* Get the calibration profile of a BSX virtual sensor */
        uint32_t actual_len = 0;
        uint16_t param_id = PARAMETER_ID;
        bhi385_bsx_algo_param_state_exg bsx_state_exg[BHI385_BSX_STATE_MAX_BLOCKS] = { { 0 } };

        for (uint8_t i = 0; i < 2; i++) /*BHI385 BASE is IMU, there's no mag block (0x0205) */
        {

            /* Get bsx states for BSX fusion parameters*/
            rslt = bhi385_bsx_algo_param_get_bsx_states(param_id,
                                                        bsx_state_exg,
                                                        sizeof(bsx_state_exg) * 20,
                                                        &actual_len,
                                                        &bhy);
            print_api_error(rslt, &bhy);

            /*lint -e10 Error 10: Lint does not understand PRIxxx */
            printf("Get Calibration profile of BSX parameter id 0x%04" PRIX16 ":\r\n", param_id);

            /*lint +e10 */

            for (uint8_t block = 0; block < BHI385_BSX_STATE_MAX_BLOCKS; block++)
            {
                bhi385_bsx_algo_param_state_exg *current_block = &bsx_state_exg[block];

                printf("Block number: %u\r\n", current_block->block_info & 0x7F);
                current_block->block_info & BHI385_BSX_STATE_TRANSFER_COMPLETE ? printf("Transfer complete\r\n") :
                printf("Transfer not complete\r\n");
                printf("Block length: %u\r\n", current_block->block_len);
                printf("Struct length: %u\r\n", current_block->struct_len);

                printf("-------------------------------------------\r\n");
                printf("Block data \r\n");
                printf("Byte hex      dec | Data\r\n");
                printf("-------------------------------------------\r\n");

                for (uint8_t i = 0; i < current_block->block_len; i++)
                {
                    if (i % 8 == 0)
                    {
                        printf("0x%06x %8d |", i, i);
                    }

                    printf("%02x ", current_block->state_data[i]);

                    if ((i + 1) % 8 == 0 || i == current_block->block_len - 1)
                    {
                        printf("\r\n");
                    }
                }

                if ((current_block->block_len == 0) ||
                    (current_block->block_info & BHI385_BSX_STATE_TRANSFER_COMPLETE) != 0)
                {
                    break;
                }

                printf("\r\n\r\n");
            }

            /* Set bsx states for physical sensors*/
            rslt = bhi385_bsx_algo_param_set_bsx_states(param_id, bsx_state_exg, &bhy);
            print_api_error(rslt, &bhy);

            /*lint -e10 Error 10: Lint does not understand PRIxxx */
            printf("Setting Calibration profile of BSX parameter id 0x%04" PRIX16 " is completed \r\n", param_id);

            /*lint +e10*/
            printf("\r\n\r\n");

            param_id = param_id + 0x2;
        }
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
