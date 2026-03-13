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
 * @file    wrist_wear_wakeup.c
 * @brief   Wrist Wear Wakeup example for the BHI385
 *
 */

#include "common.h"
#include "bhi385_parse.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite_Klio_cyclic.fw.h"

#define MAX_EXECUTE_TIME  UINT8_C(5)

int main(void)
{
    enum bhi385_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi385_dev bhy;
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    uint8_t boot_status = 0;
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };
    uint8_t loop = 0;

#ifdef BHI385_USE_I2C
    intf = BHI385_I2C_INTERFACE;
#else
    intf = BHI385_SPI_INTERFACE;
#endif

    setup_interfaces(true, intf); /* Perform a power on reset */

    init_sensor(&bhy, intf);

    setup_host_int_ctrl(&bhy);

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

        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT, bhi385_parse_meta_event, NULL, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SYS_ID_META_EVENT_WU, bhi385_parse_meta_event, NULL, &bhy);
        print_api_error(rslt, &bhy);
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_WRIST_WEAR_LP_WU,
                                                   bhi385_parse_wrist_wear_wakeup_data,
                                                   NULL,
                                                   &bhy);
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

    /* Read out data measured at 100Hz */
    sensor_conf.sample_rate = 100.0f; /* Read out data measured at 100Hz */
    sensor_conf.latency = 0; /* Report immediately */
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_WRIST_WEAR_LP_WU, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_WRIST_WEAR_LP_WU), sensor_conf.sample_rate);

    while (rslt == BHI385_OK && loop < MAX_EXECUTE_TIME)
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