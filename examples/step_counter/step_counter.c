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
 * @file    step_counter.c
 * @brief   Step counter example for the BHI385
 *
 */

#include <stdio.h>
#include "bhi385.h"
#include "bhi385_parse.h"
#include "common.h"
#include "bhi385_virtual_sensor_conf_param.h"
#include "bhi385_event_data.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite_Klio_cyclic.fw.h"

/**
* @brief Function to parse step counter format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_step_counter(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

/**
* @brief Function to parse step detector format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_step_detector(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

#define PARSE_DATA_WINDOW_SIZE  UINT16_C(15000)

int main(void)
{
    enum bhi385_intf intf;
    uint16_t version = 0;
    int8_t rslt;
    struct bhi385_dev bhy;
    uint8_t work_buffer[WORK_BUFFER_SIZE] = { 0 };
    uint8_t boot_status = 0;
    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };

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
        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_STC_LP, parse_step_counter, NULL, &bhy);
        print_api_error(rslt, &bhy);

        rslt = bhi385_register_fifo_parse_callback(BHI385_SENSOR_ID_STD_LP, parse_step_detector, NULL, &bhy);
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

    /* Enable step counter and step detector virtual sensors */
    sensor_conf.sample_rate = 1.0f; /* Read out data measured at 1Hz */
    sensor_conf.latency = 0; /* Report immediately */

    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_STC_LP, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_STC_LP), sensor_conf.sample_rate);

    rslt = bhi385_virtual_sensor_conf_param_set_cfg(BHI385_SENSOR_ID_STD_LP, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);
    printf("Enable %s at %.2fHz.\r\n", get_sensor_name(BHI385_SENSOR_ID_STD_LP), sensor_conf.sample_rate);

    uint32_t curr_ts;
    uint32_t start_ts = coines_get_millis();
    do
    {
        curr_ts = coines_get_millis();

        if (get_interrupt_status())
        {
            /* Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);
        }
    } while (rslt == BHI385_OK && (curr_ts - start_ts) < PARSE_DATA_WINDOW_SIZE);

    close_interfaces(intf);

    return rslt;
}

/**
* @brief Function to parse 32-bit scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void parse_step_counter(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;

    if (!callback_info)
    {
        printf("Null reference\r\n");

        return;
    }

    data = BHI385_LE2U32(callback_info->data_ptr);

    uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
    s = (uint32_t)(timestamp / UINT64_C(1000000000));
    ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

#ifndef PC
    printf("SID: %u; T: %lu .%09lu; %u\r\n", callback_info->sensor_id, s, ns, data);
#else
    printf("SID: %u; T: %u .%09u; %u\r\n", callback_info->sensor_id, s, ns, data);
#endif
}

void parse_step_detector(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
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

#ifndef PC
    printf("SID: %u; T: %lu .%09lu: Step detected\r\n", callback_info->sensor_id, s, ns);
#else
    printf("SID: %u; T: %u .%09u: Step detected\r\n", callback_info->sensor_id, s, ns);
#endif
}