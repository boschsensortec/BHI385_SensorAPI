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
 * @file    multi_tap_param.c
 * @brief   Multi tap example for the BHY
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "bhi385.h"
#include "bhi385_parse.h"
#include "common.h"
#include "bhi385_multi_tap_param.h"
#include "bhi385_virtual_sensor_conf_param.h"
#include "bhi385_event_data.h"

#include "bhi385/Bosch_Shuttle3_BHI385_bsxsam_lite.fw.h"

#define WORK_BUFFER_SIZE    2048

#define NUM_TAP_LOOP_COUNT  (5U)

/*! Sensor ID for Multi tap sensor */
#define CUSTOM_SENSOR_ID    BHI385_SENSOR_ID_MULTI_TAP

/*!
 * @brief Output of the multi tap data is parsed for printing
 * @param[in] callback_info fifo data available here
 * @param[in] callback_ref
 *
 * @return  void
 *
 */
static void parse_multitap(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref);

static void print_api_error(int8_t rslt, struct bhi385_dev *dev);
static void upload_firmware(uint8_t boot_stat, struct bhi385_dev *dev);

enum bhi385_intf intf;

int main(void)
{
    uint8_t chip_id = 0;
    uint16_t version = 0;
    int8_t rslt;
    uint8_t loop_cnt = 0U;
    struct bhi385_dev bhy;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    struct bhi385_virtual_sensor_conf_param_conf sensor_conf = { 0 };

    uint8_t work_buffer[WORK_BUFFER_SIZE];

    bhi385_event_data_multi_tap buffer[8] = { BHI385_NO_TAP };
    bhi385_event_data_multi_tap multitap_setting = BHI385_TRIPLE_DOUBLE_SINGLE_TAP;
    bhi385_multi_tap_param_detector multitap_cnfg = { { 0U } };

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

        /*! Registering the callback functions */
        rslt = bhi385_register_fifo_parse_callback(CUSTOM_SENSOR_ID, parse_multitap, NULL, &bhy);
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

    /*! Update the callback table to enable parsing of sensor data */
    rslt = bhi385_update_virtual_sensor_list(&bhy);
    print_api_error(rslt, &bhy);

    printf("--- Multi tap log start ---\r\n");

    rslt = bhi385_multi_tap_param_get_config(buffer, &bhy);
    print_api_error(rslt, &bhy);
    printf("Multi Tap Info : %s\r\n", bhi385_event_data_multi_tap_string_out[buffer[0]]);

    rslt = bhi385_multi_tap_param_set_config(&multitap_setting, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_multi_tap_param_detector_get_config(&multitap_cnfg, &bhy);
    print_api_error(rslt, &bhy);

    printf("Single Tap CNFG : %s\r\n",
           (((buffer[0] & (uint8_t)BHI385_SINGLE_TAP) == (uint8_t)BHI385_SINGLE_TAP) ? "Enabled" : "Disabled"));
    printf("    \t\t -<axis_sel> : %d\r\n", multitap_cnfg.stap_setting.as_s.axis_sel);
    printf("    \t\t -<wait_for_timeout> : %d\r\n", multitap_cnfg.stap_setting.as_s.wait_for_timeout);
    printf("    \t\t -<max_pks_for_tap> : %d\r\n", multitap_cnfg.stap_setting.as_s.max_peaks_for_tap);
    printf("    \t\t -<mode> : %d\r\n", multitap_cnfg.stap_setting.as_s.mode);
    printf("Double Tap CNFG : %s\r\n",
           (((buffer[0] & (uint8_t)BHI385_DOUBLE_TAP) == (uint8_t)BHI385_DOUBLE_TAP) ? "Enabled" : "Disabled"));
    printf("    \t\t -<tap_peak_thrs> : %d\r\n", multitap_cnfg.dtap_setting.as_s.tap_peak_thres);
    printf("    \t\t -<max_ges_dur> : %d\r\n", multitap_cnfg.dtap_setting.as_s.max_gesture_dur);
    printf("Triple Tap CNFG : %s\r\n",
           (((buffer[0] & (uint8_t)BHI385_TRIPLE_TAP) == (uint8_t)BHI385_TRIPLE_TAP) ? "Enabled" : "Disabled"));
    printf("    \t\t -<max_dur_bw_pks> : %d\r\n", multitap_cnfg.ttap_setting.as_s.max_dur_between_peaks);
    printf("    \t\t -<tap_shock_settl_dur> : %d\r\n", multitap_cnfg.ttap_setting.as_s.tap_shock_settling_dur);
    printf("    \t\t -<min_quite_dur_bw_taps> : %d\r\n", multitap_cnfg.ttap_setting.as_s.min_quite_dur_between_taps);
    printf("    \t\t -<quite_time_after_ges> : %d\r\n", multitap_cnfg.ttap_setting.as_s.quite_time_after_gesture);

    multitap_cnfg.stap_setting.as_s.mode = 0; /* Sensitive mode*/
    rslt = bhi385_multi_tap_param_detector_set_config(&multitap_cnfg, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_multi_tap_param_detector_get_config(&multitap_cnfg, &bhy);
    printf("Multi tap mode after setting : %d\r\n", multitap_cnfg.stap_setting.as_s.mode);
    print_api_error(rslt, &bhy);

    /*! Setting the Sampling frequency and latency time */
    sensor_conf.sample_rate = 100.0; /*! Read out data measured at 100Hz */
    sensor_conf.latency = 0; /*! Report immediately */
    rslt = bhi385_virtual_sensor_conf_param_set_cfg(CUSTOM_SENSOR_ID, &sensor_conf, &bhy);
    print_api_error(rslt, &bhy);

    rslt = bhi385_virtual_sensor_conf_param_get_cfg(CUSTOM_SENSOR_ID, &sensor_conf, &bhy);

    /*lint -e10 Error 10: Lint does not understand PRIxxx */
    printf("Multi tap sensor ID=%d, rate=%.2fHz,latency=%" PRIu32 "\r\n",
           CUSTOM_SENSOR_ID,
           sensor_conf.sample_rate,
           sensor_conf.latency);

    /*lint +e10 */

    /*! Data from the FIFO is read and the relevant callbacks if registered are called */
    while ((rslt == BHI385_OK) && (loop_cnt < NUM_TAP_LOOP_COUNT))
    {
        if (get_interrupt_status())
        {
            /*! Data from the FIFO is read and the relevant callbacks if registered are called */
            rslt = bhi385_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, &bhy);
            print_api_error(rslt, &bhy);

            loop_cnt++;
        }
    }

    printf("--- Multi tap log stop ---\r\n");
    close_interfaces(intf);

    return rslt;
}

static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

static void parse_multitap(const struct bhi385_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint32_t s, ns;
    uint64_t tns;

    bhi385_event_data_multi_tap multitap_data = BHI385_NO_TAP;

    if (callback_info->data_size != 2) /*! Check for a valid payload size. Includes sensor ID */
    {
        printf(" ERRORVAL\n");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhi385_event_data_multi_tap_parsing(callback_info->data_ptr, (uint8_t *)&multitap_data);

    /*lint -e10 Error 10: Lint does not understand PRIxxx */
    printf("SID: %u; T: %" PRIu32 ".%09" PRIu32 "; %s; \r\n",
           callback_info->sensor_id,
           s,
           ns,
           bhi385_event_data_multi_tap_string_out[multitap_data]);

    /*lint +e10 */
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
